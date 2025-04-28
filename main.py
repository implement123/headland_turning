def _init_(self):
    self.last_display_time = 0
    self.display_interval = 0.2  # Seconds between console updates
    self.lock = threading.Lock()
    
def print_status(self, message: str, clear: bool = False) -> None:
    current_time = time.time()
    with self.lock:
        if current_time - self.last_display_time >= self.display_interval:
            if clear:
                sys.stdout.write("\033[K")
                sys.stdout.write("\r")
            
            timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            sys.stdout.write(f"\r[{timestamp}] {message}")
            sys.stdout.flush()
            self.last_display_time = current_time

def print_motor_action(self, action: str, direction: str, steps: int) -> None:
    self.print_status(f"🔄 MOTOR: {action.upper()} - {direction} for {steps} steps", clear=True)

def print_encoder_value(self, angle: float, previous: float = None) -> None:
    if previous is not None:
        change = angle - previous
        direction = "⬆️" if change > 0 else "⬇️" if change < 0 else "➡️"
        self.print_status(f"🎮 ENCODER: {angle:.2f}° ({direction} {abs(change):.2f}°)", clear=True)
    else:
        self.print_status(f"🎮 ENCODER: {angle:.2f}°", clear=True)

def print_implement_status(self, status: str) -> None:
    icon = "⬆️" if status.lower() == "lifted" else "⬇️" if status.lower() == "lowered" else "🔄"
    self.print_status(f"🚜 IMPLEMENT: {status}", clear=True)
    
def print_separator(self) -> None:
    print("\n" + "=" * 50)
