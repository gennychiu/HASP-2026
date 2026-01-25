import threading
import time

class KeyboardThread(threading.Thread):
    def __init__(self, callback=None, name='keyboard-input-thread'):
        self.callback = callback
        super(KeyboardThread, self).__init__(name=name, daemon=True)
        self.start()

    def run(self):
        while True:
            # input() blocks until Enter is pressed
            user_input = input()
            if self.callback:
                self.callback(user_input)

def handle_input(inp):
    print(f"You entered: {inp}")

# Start the keyboard thread
kthread = KeyboardThread(handle_input)

# Main program loop
counter = 0
while True:
    print(f"Main program running... Counter: {counter}")
    counter += 1
    time.sleep(1)