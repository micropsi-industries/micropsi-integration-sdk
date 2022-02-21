import struct
from enum import IntEnum
from typing import Tuple

API_VERSION = 1
API_MARK = "MRSI".encode()
HEADER_FORMAT = "!4sIII"
HEADER_BYTES = 16


class MessageType(IntEnum):
    """
    Command types as defined in the mirai binary api.
    """
    GetBoxMetadata = 1
    GetTrainedSkills = 2
    ExecuteSkill = 3
    PrepareSkillAsync = 4
    GetResult = 5
    GetLastEndstateValues = 6
    GetExceptionMessage = 7
    FAILURE = 8


def unpack_header(message: bytes) -> Tuple[str, int, MessageType, int]:
    """
    Returns:
        mark, api_version, message_type, message_bytes
    """
    if len(message) < HEADER_BYTES:
        raise Exception(f"Not enough bytes in header. Wanted {HEADER_BYTES}, got {len(message)}.")
    mark, api_version, message_type, message_bytes = struct.unpack(HEADER_FORMAT,
                                                                   message[:HEADER_BYTES])
    return mark, api_version, MessageType(message_type), message_bytes


class Results(IntEnum):
    """
    Skill execution results.
    """
    NoResult = 0
    Speed = 1
    Force = 2
    Visual = 3
    Timeout = 4


REQUEST_MESSAGES = {
    MessageType.GetBoxMetadata: b'MRSI\x00\x00\x00\x01\x00\x00\x00\x01\x00\x00\x00\x10',
    MessageType.GetTrainedSkills: b'MRSI\x00\x00\x00\x01\x00\x00\x00\x02\x00\x00\x00\x10',
    MessageType.ExecuteSkill: b'MRSI\x00\x00\x00\x01\x00\x00\x00\x03\x00\x00\x00\x14\x00\x00\x00*',
    MessageType.PrepareSkillAsync: b'MRSI\x00\x00\x00\x01\x00\x00\x00\x04\x00\x00\x00\x14\x00\x00'
                                   b'\x00*',
    MessageType.GetResult: b'MRSI\x00\x00\x00\x01\x00\x00\x00\x05\x00\x00\x00\x14\x00\x00\x00*',
    MessageType.GetLastEndstateValues: b'MRSI\x00\x00\x00\x01\x00\x00\x00\x06\x00\x00\x00\x14\x00'
                                       b'\x00\x00*',
    MessageType.GetExceptionMessage: b'MRSI\x00\x00\x00\x01\x00\x00\x00\x07\x00\x00\x00\x14\x00'
                                     b'\x00\x00*',
}

RESPONSE_MESSAGES = {
    MessageType.GetBoxMetadata: b'MRSI\x00\x00\x00\x01\x00\x00\x00\x01\x00\x00\x00\x18\x00\x00'
                                b'\x00{\x00\x00\x00\x02',
    MessageType.GetTrainedSkills: b'MRSI\x00\x00\x00\x01\x00\x00\x00\x02\x00\x00\x00A\x00\x00\x00'
                                  b'\x02\x00\x00\x00\x17\x00\x00\x00\x0cmotion '
                                  b'skill\x00\x00\x00*\x00\x00\x00\x11positioning skill',
    MessageType.ExecuteSkill: b'MRSI\x00\x00\x00\x01\x00\x00\x00\x03\x00\x00\x00\x11\x01',
    MessageType.PrepareSkillAsync: b'MRSI\x00\x00\x00\x01\x00\x00\x00\x04\x00\x00\x00\x11\x01',
    MessageType.GetResult: b'MRSI\x00\x00\x00\x01\x00\x00\x00\x05\x00\x00\x00\x14\x00\x00\x00\x02',
    MessageType.GetLastEndstateValues:
        b'MRSI\x00\x00\x00\x01\x00\x00\x00\x06\x00\x00\x00\x1c>L\xcc\xcd>\x99\x99\x9a>\xcc\xcc\xcd',
    MessageType.GetExceptionMessage: b'MRSI\x00\x00\x00\x01\x00\x00\x00\x07\x00\x00\x00%\x00\x00'
                                     b'\x00\x11exception message',
    MessageType.FAILURE: b'MRSI\x00\x00\x00\x01\x00\x00\x00\x08\x00\x00\x00%\x00\x00\x00'
                         b'\x11something went wrong'
}

RESULT_MESSAGES = {
    Results.NoResult: b'MRSI\x00\x00\x00\x01\x00\x00\x00\x05\x00\x00\x00\x14\x00\x00\x00\x00',
    Results.Visual: b'MRSI\x00\x00\x00\x01\x00\x00\x00\x05\x00\x00\x00\x14\x00\x00\x00\x03',
}
