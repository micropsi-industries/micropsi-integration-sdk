#!/usr/bin/env python3
import argparse
import logging
import os
import socket
import time

from micropsi_integration_sdk.dev_schema import (
    MessageType,
    REQUEST_MESSAGES_V1,
    REQUEST_MESSAGES_V2,
)

logger = logging.getLogger("mirai-dev-client")


REQUEST_MESSAGES = {
    1: REQUEST_MESSAGES_V1,
    2: REQUEST_MESSAGES_V2,
}


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
    parser.add_argument("--count", type=int, default=1,
                        help="Send the command COUNT times, reusing the connection.")
    parser.add_argument("--period", type=float, default=1,
                        help="Wait PERIOD seconds between sent messages.")
    parser.add_argument("--api-version", type=int, default=1, choices=REQUEST_MESSAGES.keys(),
                        help="Format messages for the given mirai binary api version.")
    parser.add_argument("command", choices=[c.name for c in iter(MessageType)
                                            if c != MessageType.FAILURE])
    return parser.parse_args()


def main():
    logging.basicConfig(level=logging.INFO)
    args = parse_args()
    server_address = args.server_address
    command = args.command
    command = getattr(MessageType, command)
    request_messages = REQUEST_MESSAGES[args.api_version]
    message = request_messages[command]
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(1)
        sock.connect((server_address, 6599))
        count = 0
        while count < args.count:
            count += 1
            logger.info("sending %s", message)
            sock.sendall(message)
            recv_until_response(sock)
            time.sleep(args.period)


class ServerDisconnected(Exception):
    pass


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
        if response == b'':
            raise ServerDisconnected

    logger.info("received, %s", response)
    return response


if __name__ == "__main__":
    main()
