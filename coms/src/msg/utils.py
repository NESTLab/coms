from coms.constants import ENCODING


MESSAGE_REGISTRY = {
    0: 'Merge',
    1: 'Ping',
}


def extract_payload_id(payload: bytes) -> int:
    s = payload.decode(ENCODING)
    if len(s) == 0:
        return -1
    return int(s[0], 10)


def get_message_type(payload: bytes) -> str or None:
    id = extract_payload_id(payload)
    if id not in MESSAGE_REGISTRY:
        return None
    return MESSAGE_REGISTRY[id]
