#!/usr/bin/env python3
import argparse
import logging
import os
import socket

from micropsi_integration_sdk.dev_schema import (
    MessageType,
    REQUEST_MESSAGES,
)

logger = logging.getLogger("mirai-dev-client")


class ArgsFormatter(argparse.ArgumentDefaultsHelpFormatter, argparse.RawTextHelpFormatter):
    pass


def parse_args():
    parser = argparse.ArgumentParser(
        formatter_class=ArgsFormatter,
        epilog=os.linesep.join([
            "Usage example:",
            "# mirai-dev-client GetBoxMetadata"])
    )
    parser.add_argument("--server-address", default="localhost",
                        help="Hostname or IP address where the mirai dev server is running.")
    parser.add_argument("command", choices=[c.name for c in iter(MessageType)
                                            if c != MessageType.FAILURE])
    return parser.parse_args()


def main():
    logging.basicConfig(level=logging.INFO)
    args = parse_args()
    server_address = args.server_address
    command = args.command
    command = getattr(MessageType, command)
    message = REQUEST_MESSAGES[command]
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(1)
        sock.connect((server_address, 6599))
        logger.info("sending %s", message)
        sock.sendall(message)
        recv_until_response(sock)


def recv_until_response(sock: socket.socket) -> bytes:
    """
    Repeatedly attempt to recv up to 1024 bytes from the socket until a response.
    """
    response = None
    while response is None:
        try:
            response = sock.recv(1024)
        except socket.timeout:
            continue
    logger.info("received, %s", response)
    return response


if __name__ == "__main__":
    main()
