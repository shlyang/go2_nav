from common.enums import CommandType
import threading
from queue import Queue

class Command:
    def __init__(self, cmd_type, params=None):
        self.type = cmd_type
        self.params = params
        
class CommandHandler:
    def __init__(self):
        self._command_queue = Queue()
        self._lock = threading.Lock()
        
    def add_command(self, command):
        """添加新命令到队列"""
        with self._lock:
            self._command_queue.put(command)
            
    def peek_command(self):
        """查看队列头部的命令但不移除"""
        with self._lock:
            if not self._command_queue.empty():
                return self._command_queue.queue[0]
            return None
            
    def pop_command(self):
        """移除并返回队列头部的命令"""
        with self._lock:
            return self._command_queue.get() if not self._command_queue.empty() else None