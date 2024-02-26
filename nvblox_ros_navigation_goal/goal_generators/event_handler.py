from watchdog.events import FileSystemEventHandler

class EventHandler(FileSystemEventHandler):
    def __init__(self):
        super().__init__()
        self.file_modified = False  # 파일 수정 여부를 추적하는 플래그

    def on_modified(self, event):
        print(f'File {event.src_path} has been modified.')
        self.file_modified = True  # 파일이 수정되면 플래그를 True로 설정