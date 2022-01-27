import socket
from typing import Tuple
from threading import current_thread, Event, Thread
from socketserver import BaseRequestHandler, TCPServer, ThreadingMixIn, ThreadingTCPServer
from coms.constants import ENCODING


class ThreadedTCPRequestHandler(BaseRequestHandler):
    def handle(self: BaseRequestHandler) -> None:
        data = str(self.request.recv(1024), ENCODING)
        cur_thread = current_thread()
        response = bytes("{}: {}".format(cur_thread.name, data), ENCODING)
        self.request.sendall(response)


class ThreadedTCPServer(ThreadingMixIn, TCPServer):
    def server_bind(self: ThreadingTCPServer) -> None:
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(self.server_address)


def client(source: Tuple[str, int], destination: Tuple[str, int], message: str) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, 25, str("tun1" + '\0').encode('utf-8'))
        # sock.bind(source)
        sock.connect(destination)
        print(sock.getpeername())
        print(sock.getsockname())
        sock.sendall(bytes(message, ENCODING))
        response = str(sock.recv(1024), ENCODING)
        print("Received: {}".format(response))
        sock.close()


def server(address: Tuple[str, int], termination_event: Event) -> None:
    server = ThreadedTCPServer(address, ThreadedTCPRequestHandler)
    with server:
        # Start a thread with the server -- that thread will then start one
        # more thread for each request
        server_thread = Thread(target=server.serve_forever)
        # Exit the server thread when the main thread terminates
        server_thread.daemon = True
        server_thread.start()
        print("Server loop running in thread:", server_thread.name)

        termination_event.wait()

        server.shutdown()
        server.socket.close()


# if __name__ == "__main__":
#     server_address = ("192.168.0.1", 8812)
#     server_address2 = ("192.168.0.3", 8812)
#     client_address = ("192.168.0.2", 8818)

#     event = Event()
#     srv = Thread(target=server, args=[server_address, event])
#     srv2 = Thread(target=server, args=[server_address2, event])

#     srv.start()
#     srv2.start()

#     client(client_address, server_address, "Hello World 1")
#     client(client_address, server_address2, "Hello World 2")
#     client(client_address, server_address, "Hello World 3")

#     event.set()
#     srv.join()
#     srv2.join()
