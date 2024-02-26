import os

current_file_path = os.path.abspath(__file__)
current_file_path = os.path.dirname(current_file_path)
current_file_path = os.path.dirname(current_file_path)
root_path = os.path.dirname(current_file_path)
goal_text_file_path = os.path.join(root_path, 'maps', 'goals.txt')