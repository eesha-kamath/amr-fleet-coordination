from abc import ABC, abstractmethod


class Transport(ABC):
    @abstractmethod
    def send(self, message):
        pass

    @abstractmethod
    def receive(self):
        pass


class InMemoryTransport(Transport):
    def __init__(self):
        self.messages = []

    def send(self, message):
        self.messages.append(message)

    def receive(self):
        messages = self.messages[:]
        self.messages.clear()
        return messages