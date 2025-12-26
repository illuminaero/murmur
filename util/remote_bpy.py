import bpy
import socket
import threading
import traceback
import contextlib
import queue
import sys

HOST = "127.0.0.1"
PORT = 5678

job_queue = queue.Queue()

# Save original streams at module load time (before any redirection)
_original_stdout = sys.__stdout__
_original_stderr = sys.__stderr__

class Job:
    def __init__(self, code):
        self.code = code
        self.output_queue = queue.Queue()
        self.done = threading.Event()

class StreamWriter:
    """File-like object that sends writes to both remote terminal and stdout."""
    def __init__(self, job, original_stream):
        self.job = job
        self.original_stream = original_stream

    def write(self, data):
        if data:
            # Send chunks of text to the socket thread (remote terminal)
            self.job.output_queue.put(data)
            # Also write to original stdout/stderr (Blender console)
            self.original_stream.write(data)
            self.original_stream.flush()
        return len(data)

    def flush(self):
        self.original_stream.flush()


def handle_client(conn, addr):
    print(f"[remote_bpy] connection from {addr}")
    try:
        data = conn.recv(10_000_000).decode("utf-8")
        if not data:
            conn.close()
            return

        job = Job(data)
        job_queue.put(job)

        # Stream output as it becomes available
        while True:
            try:
                chunk = job.output_queue.get(timeout=0.1)
            except queue.Empty:
                # If job is finished and no more output, exit
                if job.done.is_set():
                    break
                continue

            if chunk is None:
                # Sentinel meaning "we're done"
                break

            conn.sendall(chunk.encode("utf-8", errors="replace"))

    finally:
        conn.close()


def server():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(5)
    print(f"[remote_bpy] listening on {HOST}:{PORT}")
    while True:
        conn, addr = s.accept()
        t = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
        t.start()


def process_jobs():
    """
    Runs on Blender's main thread via bpy.app.timers.
    This is the ONLY place we touch bpy or exec the code.
    """
    try:
        job = job_queue.get_nowait()
    except queue.Empty:
        # Nothing to do right now; check again later
        return 0.05

    # persistent namespace, so you can define helpers once and reuse
    global _remote_ns
    try:
        _remote_ns
    except NameError:
        _remote_ns = {"bpy": bpy}

    # Create writers that output to both remote and local (using original streams)
    stdout_writer = StreamWriter(job, _original_stdout)
    stderr_writer = StreamWriter(job, _original_stderr)

    with contextlib.redirect_stdout(stdout_writer), contextlib.redirect_stderr(stderr_writer):
        try:
            code = compile(job.code, "<remote_bpy>", "exec")
            exec(code, _remote_ns, _remote_ns)
        except Exception:
            traceback.print_exc()

    # Signal end of stream
    job.output_queue.put(None)
    job.done.set()

    return 0.05  # schedule next check soon


threading.Thread(target=server, daemon=True).start()
bpy.app.timers.register(process_jobs)

print("[remote_bpy] streaming server started")
