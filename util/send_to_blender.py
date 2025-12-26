import socket
import sys
from pathlib import Path

HOST = "127.0.0.1"
PORT = 5678

def main(path):
    code = Path(path).read_text(encoding="utf-8")

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    s.sendall(code.encode("utf-8"))
    s.shutdown(socket.SHUT_WR)

    # Stream output line-by-line
    while True:
        data = s.recv(4096)
        if not data:
            break
        sys.stdout.write(data.decode("utf-8", errors="replace"))
        sys.stdout.flush()

    s.close()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python send_to_blender.py path/to/script.py")
        sys.exit(1)
    main(sys.argv[1])
