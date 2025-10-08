import struct
from typing import Optional


class Message:
    def __init__(self, data: bytes):
        self.id =  struct.unpack('B', data[0:1])[0]
        self.data = struct.unpack('<I', data[1:5])[0]

    def __str__(self):
        return f"id: {self.id}, data: {self.data}"

def decode(data: bytes) -> int:
    # first byte is id
    msgs = data.split(b'\xFF\xFF\xFF\x00')
    i = 0
    for d in msgs:
        print(Message(d))
        i += 1
    return i

class CircularBuffer:
    def __init__(self, size: int):
        self.buffer = [b'\x00'for i in range(size)]
        self.head = 0
        self.size = 0
        self.tail = 0

    def append(self, item: Optional[bytes]):
        if item is None:
            return
        items = item.split(b'\xff\xff\xff\x00')
        for item in items:
            if len(item) == 0:
                continue
            self.buffer[self.head] = item
            self.head = (self.head + 1) % len(self.buffer)
            self.size += 1
        if self.size > len(self.buffer):
            self.size = len(self.buffer)

    def pop(self) -> Optional[bytes]:
        if self.size == 0:
            return None
        item = self.buffer[self.tail]
        self.tail = (self.tail + 1) % len(self.buffer)
        self.size -= 1
        return item
