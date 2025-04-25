# headland_turning
import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt
import json
import logging
import argparse
import configparser
from enum import Enum
from typing import Optional, Dict, Any
import sys
from contextlib import contextmanager
import threading
import datetime

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('headland_management.log')
    ]
)
logger = logging.getLogger('headland_management')

class ConsoleDisplay:
    """Manages console output for real-time feedback to users."""
    
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

class HeadlandPattern(Enum):
    SIDE_BY_SIDE = 0
    CONTINUOUS = 1
    GATHERING = 2
    CASTING = 3
    
    @classmethod
    def get_name(cls, value: int) -> str:
        names = {
            cls.SIDE_BY_SIDE.value: "Side by Side",
            cls.CONTINUOUS.value: "Continuous",
            cls.GATHERING.value: "Gathering",
            cls.CASTING.value: "Casting"
        }
        return names.get(value, "Unknown")

    @classmethod
    def list_patterns(cls) -> None:
        for pattern in cls:
            print(f"{pattern.value}: {cls.get_name(pattern.value)}")

class MotorState(Enum):
    WAITING_FOR_LIFT = "waiting_for_lift"
    WAITING_FOR_INDEX = "waiting_for_index"
    WAITING_FOR_LOWER = "waiting_for_lower"

class MotorController:
    """Controls stepper motor for implement lifting, lowering, and indexing."""
    
    def _init_(self, dir_pin: int, step_pin: int, steps_per_rotation: int, 
                 simulate: bool = False, step_delay: float = 0.005, console: ConsoleDisplay = None):
        self.dir_pin = dir_pin  
        self.step_pin = step_pin
        self.steps_per_rotation = steps_per_rotation
        self.cw_direction = GPIO.HIGH
        self.ccw_direction = GPIO.LOW
        self.simulate = simulate
        self.step_delay = step_delay
        self.console = console or ConsoleDisplay()
        
        if not self.simulate:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.dir_pin, GPIO.OUT)
                GPIO.setup(self.step_pin, GPIO.OUT)
                GPIO.output(self.dir_pin, GPIO.LOW)
                GPIO.output(self.step_pin, GPIO.LOW)
                logger.info(f"Motor controller initialized with DIR_PIN={dir_pin}, STEP_PIN={step_pin}")
                self.console.print_status(f"Motor controller initialized with DIR_PIN={dir_pin}, STEP_PIN={step_pin}")
            except Exception as e:
                logger.error(f"Failed to initialize GPIO: {e}")
                self.console.print_status(f"ERROR: Failed to initialize GPIO: {e}")
                raise
        else:
            logger.info("Running in simulation mode - no GPIO operations will be performed")
            self.console.print_status("Running in simulation mode - no GPIO operations will be performed")
    
    def rotate(self, direction: int, steps: int, action: str) -> bool:
        direction_str = 'CW' if direction == self.cw_direction else 'CCW'
        logger.info(f"Rotating motor {direction_str} for {steps} steps ({action}).")
        self.console.print_motor_action(action, direction_str, steps)
        
        if self.simulate:
            print()
            for i in range(10):
                progress = "█" * (i + 1) + "□" * (9 - i)
                sys.stdout.write(f"\r[SIMULATING] {progress} {(i+1)*10}%")
                sys.stdout.flush()
                time.sleep(steps * self.step_delay * 2 / 10)
            print("\nSimulation complete!")
            return True
            
        try:
            GPIO.output(self.dir_pin, direction)
            step_indicators = ["|", "/", "-", "\\"]
            for step in range(steps):
                GPIO.output(self.step_pin, GPIO.HIGH)
                time.sleep(self.step_delay)
                GPIO.output(self.step_pin, GPIO.LOW)
                time.sleep(self.step_delay)
                if step % (steps // min(10, steps) or 1) == 0:
                    indicator = step_indicators[(step // (steps // min(10, steps) or 1)) % 4]
                    progress_pct = int(step / steps * 100)
                    self.console.print_status(
                        f"🔄 MOTOR: {action.upper()} - {direction_str} {indicator} {progress_pct}%", 
                        clear=True
                    )
            
            self.console.print_status(f"🔄 MOTOR: {action.upper()} - {direction_str} COMPLETE", clear=True)
            return True
        except Exception as e:
            logger.error(f"Error in rotate_motor: {e}")
            self.console.print_status(f"ERROR: Failed to rotate motor: {e}", clear=True)
            return False
            
    def lift(self) -> bool:
        self.console.print_implement_status("LIFTING")
        result = self.rotate(self.cw_direction, self.steps_per_rotation, "lift")
        if result:
            self.console.print_implement_status("LIFTED")
        return result
        
    def lower(self) -> bool:
        self.console.print_implement_status("LOWERING")
        result = self.rotate(self.ccw_direction, self.steps_per_rotation, "lower")
        if result:
            self.console.print_implement_status("LOWERED")
        return result
        
    def index(self) -> bool:
        self.console.print_implement_status("INDEXING")
        result = self.rotate(self.cw_direction, self.steps_per_rotation, "index")
        if result:
            self.console.print_implement_status("INDEXED")
        return result
        
    def cleanup(self) -> None:
        if not self.simulate:
            GPIO.cleanup()
            logger.info("GPIO cleanup completed.")
            self.console.print_status("GPIO cleanup completed.")

class MQTTHandler:
    """Handles MQTT communication for steering angle data."""
    
    def _init_(self, broker: str, port: int, topic: str, client_id: Optional[str] = None, 
                 console: ConsoleDisplay = None):
        self.broker = broker
        self.port = port
        self.topic = topic
        self.steering_angle = 0
        self.previous_angle = 0
        self.client_id = client_id or f"headland_manager_{int(time.time())}"
        self.client = mqtt.Client(client_id=self.client_id)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
        self.connected = False
        self.last_message_time = 0
        self.console = console or ConsoleDisplay()
        self.reconnect_interval = 5  # Seconds before attempting reconnect
        
    def connect(self) -> bool:
        try:
            logger.info(f"Connecting to MQTT broker at {self.broker}:{self.port}...")
            self.console.print_status(f"Connecting to MQTT broker at {self.broker}:{self.port}...")
            self.client.connect(self.broker, self.port, 60)
            self.client.loop_start()
            start_time = time.time()
            while not self.connected and time.time() - start_time < 5:
                time.sleep(0.1)
            
            if self.connected:
                self.console.print_status(f"✅ Connected to MQTT broker at {self.broker}")
            else:
                self.console.print_status(f"❌ Failed to connect to MQTT broker at {self.broker}")
            
            return self.connected
        except Exception as e:
            logger.error(f"MQTT connection failed: {e}")
            self.console.print_status(f"❌ MQTT connection failed: {e}")
            return False
            
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            logger.info(f"Connected to MQTT broker with code: {rc}")
            client.subscribe(self.topic)
            self.connected = True
            self.console.print_status(f"✅ MQTT: Subscribed to {self.topic}")
        else:
            logger.error(f"Failed to connect to MQTT with code: {rc}")
            self.console.print_status(f"❌ MQTT: Failed to connect with code: {rc}")
            self.connected = False
            
    def on_disconnect(self, client, userdata, rc):
        logger.warning(f"Disconnected from MQTT broker with code: {rc}")
        self.console.print_status(f"❌ MQTT: Disconnected with code: {rc}")
        self.connected = False
        
    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            self.previous_angle = self.steering_angle
            new_angle = data.get('position', self.steering_angle)
            direction = data.get('direction', 'Unknown')
            
            if isinstance(new_angle, (int, float)):
                self.steering_angle = new_angle
                self.last_message_time = time.time()
                logger.debug(f"Received - Steering Angle: {self.steering_angle} deg, Direction: {direction}")
                self.console.print_encoder_value(self.steering_angle, self.previous_angle)
            else:
                logger.warning(f"Invalid steering angle data: {new_angle}")
                self.console.print_status(f"⚠️ MQTT: Invalid steering angle data: {new_angle}")
        except json.JSONDecodeError:
            logger.error(f"Invalid JSON payload: {msg.payload}")
            self.console.print_status(f"❌ MQTT: Invalid JSON payload")
        except Exception as e:
            logger.error(f"Error processing MQTT message: {e}")
            self.console.print_status(f"❌ MQTT: Error processing message: {e}")
            
    def get_steering_angle(self) -> float:
        if time.time() - self.last_message_time > 5 and self.last_message_time > 0:
            logger.warning("No steering data received for over 5 seconds")
            self.console.print_status("⚠️ MQTT: No steering data received for over 5 seconds")
            if not self.connected:
                self.reconnect()
        return self.steering_angle
        
    def reconnect(self) -> None:
        if not self.connected:
            logger.info("Attempting to reconnect to MQTT broker...")
            self.console.print_status("Attempting to reconnect to MQTT broker...")
            try:
                self.client.reconnect()
            except Exception as e:
                logger.error(f"Reconnect failed: {e}")
                self.console.print_status(f"❌ Reconnect failed: {e}")
                
    def disconnect(self) -> None:
        try:
            self.client.loop_stop()
            self.client.disconnect()
            logger.info("MQTT disconnected.")
            self.console.print_status("MQTT disconnected.")
        except Exception as e:
            logger.error(f"Error disconnecting MQTT: {e}")
            self.console.print_status(f"Error disconnecting MQTT: {e}")
            
    def publish_status(self, status: Dict[str, Any]) -> bool:
        if not self.connected:
            logger.warning("Cannot publish: Not connected to MQTT broker")
            return False
            
        try:
            status_topic = f"{self.topic}/status"
            payload = json.dumps(status)
            result = self.client.publish(status_topic, payload)
            if result.rc == 0:
                logger.debug(f"Published status to {status_topic}")
                return True
            else:
                logger.warning(f"Failed to publish status, error code: {result.rc}")
                return False
        except Exception as e:
            logger.error(f"Error publishing status: {e}")
            return False

class HeadlandManager:
    """Manages implement position based on steering angle and selected pattern."""
    
    def _init_(self, motor_controller: MotorController, mqtt_handler: MQTTHandler, 
                 config: configparser.ConfigParser, console: ConsoleDisplay = None):
        self.motor = motor_controller
        self.mqtt = mqtt_handler
        self.console = console or ConsoleDisplay()
        
        # Configuration
        self.steering_threshold = config.getfloat('Thresholds', 'steering_threshold', fallback=25)
        self.gathering_upper = config.getfloat('Thresholds', 'gathering_upper_threshold', fallback=15)
        self.gathering_lower = config.getfloat('Thresholds', 'gathering_lower_threshold', fallback=10)
        self.hysteresis = config.getfloat('Thresholds', 'hysteresis', fallback=1)
        self.high_threshold = config.getfloat('Thresholds', 'high_threshold', fallback=15)
        self.low_threshold = config.getfloat('Thresholds', 'low_threshold', fallback=5)
        self.casting_lower_threshold = config.getfloat('Thresholds', 'casting_lower_threshold', fallback=15)
        self.casting_upper_threshold = config.getfloat('Thresholds', 'casting_upper_threshold', fallback=25)
        
        # State variables
        self.implement_lifted = False
        self.state = MotorState.WAITING_FOR_LIFT
        self.turn_count = 0
        self.step_counter = 0
        self.last_status_time = 0
        self.last_steering_angle = 0
        self.previous_angle = 0
        self.status_interval = 5
        self.need_index = False
        
        logger.info("Headland manager initialized with thresholds: "
                   f"steering={self.steering_threshold}, "
                   f"gathering_upper={self.gathering_upper}, "
                   f"gathering_lower={self.gathering_lower}, "
                   f"high={self.high_threshold}, "
                   f"low={self.low_threshold}, "
                   f"casting_lower={self.casting_lower_threshold}, "
                   f"casting_upper={self.casting_upper_threshold}")
        self.console.print_status("Headland manager initialized with thresholds: "
                     f"steering={self.steering_threshold}, "
                     f"gathering_upper={self.gathering_upper}, "
                     f"gathering_lower={self.gathering_lower}, "
                     f"high={self.high_threshold}, "
                     f"low={self.low_threshold}, "
                     f"casting_lower={self.casting_lower_threshold}, "
                     f"casting_upper={self.casting_upper_threshold}")
        self.console.print_separator()
        
    def run_algorithm(self, pattern: int) -> None:
        steering_angle = self.mqtt.get_steering_angle()
        
        if abs(steering_angle - self.last_steering_angle) >= 0.5:
            self.console.print_encoder_value(steering_angle, self.last_steering_angle)
            self.last_steering_angle = steering_angle
        
        logger.debug(f"Pattern: {HeadlandPattern.get_name(pattern)}, State: {self.state.value}, "
                   f"Steering Angle: {steering_angle} deg, Lifted: {self.implement_lifted}, "
                   f"Step Counter: {self.step_counter}")
        
        if time.time() - self.last_status_time > self.status_interval:
            self._publish_status(pattern, steering_angle)
            self.last_status_time = time.time()
        
        if pattern == HeadlandPattern.SIDE_BY_SIDE.value:
            self._side_by_side_algorithm(steering_angle)
        elif pattern == HeadlandPattern.CONTINUOUS.value:
            self._continuous_algorithm(steering_angle)
        elif pattern == HeadlandPattern.GATHERING.value:
            self._gathering_algorithm(steering_angle)
        elif pattern == HeadlandPattern.CASTING.value:
            self._casting_algorithm(steering_angle)
        else:
            logger.warning(f"Unknown pattern: {pattern}")
    
    def _publish_status(self, pattern: int, steering_angle: float) -> None:
        status = {
            "pattern": HeadlandPattern.get_name(pattern),
            "state": self.state.value,
            "steering_angle": steering_angle,
            "implement_lifted": self.implement_lifted,
            "turn_count": self.turn_count,
            "step_counter": self.step_counter,
            "timestamp": time.time()
        }
        self.mqtt.publish_status(status)
    
    def _side_by_side_algorithm(self, steering_angle: float) -> None:
        abs_angle = abs(steering_angle)
        
        if not self.implement_lifted and abs_angle > self.steering_threshold:
            logger.info(f"Side by Side: Lifting implement (angle={abs_angle}°)")
            self.console.print_status(f"🚜 Lifting at {abs_angle:.1f}°")
            if self.motor.lift():
                self.implement_lifted = True
                self.state = MotorState.WAITING_FOR_INDEX
                self.console.print_status("🚜 IMPLEMENT: LIFTED and waiting for index position", clear=True)
        
        elif self.implement_lifted and self.state == MotorState.WAITING_FOR_INDEX:
            if abs_angle < self.steering_threshold - self.hysteresis:
                logger.info(f"Side by Side: Indexing implement (angle={abs_angle}°)")
                self.console.print_status(f"🚜 Indexing at {abs_angle:.1f}°")
                if self.motor.index():
                    self.state = MotorState.WAITING_FOR_LOWER
                    self.console.print_status("🚜 IMPLEMENT: INDEXED and waiting to lower", clear=True)
        
        elif self.implement_lifted and self.state == MotorState.WAITING_FOR_LOWER:
            if abs_angle < self.steering_threshold - self.hysteresis:
                logger.info(f"Side by Side: Lowering implement (angle={abs_angle}°)")
                self.console.print_status(f"🚜 Lowering at {abs_angle:.1f}°") 
                if self.motor.lower():
                    self.implement_lifted = False
                    self.state = MotorState.WAITING_FOR_LIFT
                    self.turn_count += 1
                    self.console.print_status(f"🚜 IMPLEMENT: LOWERED - Turn complete ({self.turn_count})", clear=True)
    
    def _continuous_algorithm(self, steering_angle: float) -> None:
        abs_angle = abs(steering_angle)
        prev_abs_angle = abs(self.previous_angle)
        
        # Detect transition above high threshold
        if prev_abs_angle < self.high_threshold and abs_angle >= self.high_threshold:
            self.step_counter += 1
            if self.step_counter % 4 == 1:  # Steps 1, 5, 9, ...
                if not self.implement_lifted:
                    logger.info(f"Continuous: Lifting and indexing at {abs_angle}° (step {self.step_counter})")
                    self.console.print_status(f"🚜 Lifting and indexing at {abs_angle:.1f}° (step {self.step_counter})")
                    if self.motor.lift():
                        if self.motor.index():
                            self.implement_lifted = True
                            self.state = MotorState.WAITING_FOR_LOWER
                        else:
                            logger.error("Failed to index motor")
                            self.console.print_status("❌ Failed to index motor")
                    else:
                        logger.error("Failed to lift motor")
                        self.console.print_status("❌ Failed to lift motor")
        
        # Detect transition below low threshold
        if prev_abs_angle > self.low_threshold and abs_angle <= self.low_threshold:
            if self.step_counter % 4 == 0:  # Steps 4, 8, 12, ...
                if self.implement_lifted:
                    logger.info(f"Continuous: Lowering implement at {abs_angle}° (step {self.step_counter})")
                    self.console.print_status(f"🚜 Lowering at {abs_angle:.1f}° (step {self.step_counter})")
                    if self.motor.lower():
                        self.implement_lifted = False
                        self.state = MotorState.WAITING_FOR_LIFT
                        self.turn_count += 1
                    else:
                        logger.error("Failed to lower motor")
                        self.console.print_status("❌ Failed to lower motor")
        
        self.previous_angle = steering_angle
    
    def _gathering_algorithm(self, steering_angle: float) -> None:
        abs_angle = abs(steering_angle)
        
        if self.state == MotorState.WAITING_FOR_LIFT and abs_angle > self.steering_threshold:
            if self.turn_count == 0 or self.turn_count == 1:
                logger.info(f"Gathering: Lifting implement at turn {self.turn_count + 1} (angle={abs_angle:.1f}°)")
                self.console.print_status(f"🚜 Lifting at turn {self.turn_count + 1}, angle {abs_angle:.1f}°")
                if self.motor.lift():
                    self.implement_lifted = True
                    self.state = MotorState.WAITING_FOR_LOWER
                    self.turn_count += 1
                    self.console.print_status(f"🚜 IMPLEMENT: LIFTED - Turn {self.turn_count}", clear=True)
            
            elif self.turn_count == 3 or (self.turn_count >= 6 and (self.turn_count - 3) % 4 == 0):
                logger.info(f"Gathering: Lifting implement at turn {self.turn_count + 1} (angle={abs_angle:.1f}°)")
                self.console.print_status(f"🚜 Lifting at turn {self.turn_count + 1}, angle {abs_angle:.1f}°")
                if self.motor.lift():
                    logger.info("Gathering: Indexing implement after lift")
                    self.console.print_status(f"🚜 Indexing after lift")
                    if self.motor.index():
                        self.implement_lifted = True
                        self.state = MotorState.WAITING_FOR_LOWER
                        self.turn_count += 1
                        self.console.print_status(f"🚜 IMPLEMENT: LIFTED and INDEXED - Turn {self.turn_count}", clear=True)
            
            else:
                logger.debug(f"Gathering: No action at turn {self.turn_count + 1} (angle={abs_angle:.1f}°)")
                self.console.print_status(f"🚜 No action at turn {self.turn_count + 1}")
                self.turn_count += 1
                self.state = MotorState.WAITING_FOR_LOWER
        
        elif self.state == MotorState.WAITING_FOR_LOWER and abs_angle < self.steering_threshold - self.hysteresis:
            if self.turn_count == 2:
                logger.info(f"Gathering: Lowering implement at turn {self.turn_count} (angle={abs_angle:.1f}°)")
                self.console.print_status(f"🚜 Lowering at turn {self.turn_count}, angle {abs_angle:.1f}°")
                if self.motor.lower():
                    self.implement_lifted = False
                    self.state = MotorState.WAITING_FOR_LIFT
                    self.turn_count += 1
                    self.console.print_status(f"🚜 IMPLEMENT: LOWERED - Turn {self.turn_count}", clear=True)
            
            elif self.turn_count >= 7 and (self.turn_count - 7) % 4 == 0:
                logger.info(f"Gathering: Lowering implement at turn {self.turn_count} (angle={abs_angle:.1f}°)")
                self.console.print_status(f"🚜 Lowering at turn {self.turn_count}, angle {abs_angle:.1f}°")
                if self.motor.lower():
                    self.implement_lifted = False
                    self.state = MotorState.WAITING_FOR_LIFT
                    self.turn_count += 1
                    self.console.print_status(f"🚜 IMPLEMENT: LOWERED - Turn {self.turn_count}", clear=True)
            
            else:
                logger.debug(f"Gathering: Turn {self.turn_count} completed (angle={abs_angle:.1f}°)")
                self.console.print_status(f"🚜 Turn {self.turn_count} completed")
                self.state = MotorState.WAITING_FOR_LIFT
    
    def _casting_algorithm(self, steering_angle: float) -> None:
        abs_angle = abs(steering_angle)
        prev_abs_angle = abs(self.previous_angle)
        
        # Lift when angle exceeds lower threshold
        if not self.implement_lifted and abs_angle > self.casting_lower_threshold:
            logger.info(f"Casting: Lifting implement at {abs_angle}°")
            self.console.print_status(f"🚜 Lifting at {abs_angle:.1f}°")
            if self.motor.lift():
                self.implement_lifted = True
                if abs_angle > self.casting_upper_threshold:
                    self.need_index = True
                    self.console.print_status("🚜 Will index when angle drops (Side-by-Side)")
                else:
                    logger.info("Casting: Indexing immediately (Continuous)")
                    self.console.print_status("🚜 Indexing immediately (Continuous)")
                    if self.motor.index():
                        self.need_index = False
                    else:
                        logger.error("Failed to index motor")
                        self.console.print_status("❌ Failed to index")
                        self.need_index = True
            else:
                logger.error("Failed to lift motor")
                self.console.print_status("❌ Failed to lift")
        
        # Monitor for upper threshold breach while lifted
        elif self.implement_lifted:
            if abs_angle > self.casting_upper_threshold:
                self.need_index = True
                self.console.print_status(f"🚜 Angle {abs_angle:.1f}° > {self.casting_upper_threshold}°, will index when angle drops")
            if abs_angle < self.casting_lower_threshold - self.hysteresis:
                if self.need_index:
                    logger.info(f"Casting: Indexing before lowering at {abs_angle}°")
                    self.console.print_status(f"🚜 Indexing before lowering at {abs_angle:.1f}°")
                    if self.motor.index():
                        self.need_index = False
                    else:
                        logger.error("Failed to index motor")
                        self.console.print_status("❌ Failed to index")
                logger.info(f"Casting: Lowering implement at {abs_angle}°")
                self.console.print_status(f"🚜 Lowering at {abs_angle:.1f}°")
                if self.motor.lower():
                    self.implement_lifted = False
                    self.state = MotorState.WAITING_FOR_LIFT
                    self.turn_count += 1
                    self.console.print_status(f"🚜 Lowered - Turn {self.turn_count}")
                else:
                    logger.error("Failed to lower motor")
                    self.console.print_status("❌ Failed to lower")
        
        self.previous_angle = steering_angle

def select_algorithm() -> int:
    console = ConsoleDisplay()
    console.print_separator()
    while True:
        print("\nSelect headland management algorithm:")
        HeadlandPattern.list_patterns()
        
        try:
            choice = int(input("\nEnter algorithm number: "))
            if choice in [pattern.value for pattern in HeadlandPattern]:
                console.print_status(f"Selected algorithm: {HeadlandPattern.get_name(choice)}")
                console.print_separator()
                return choice
            else:
                print("Invalid choice. Please enter a valid number.")
        except ValueError:
            print("Invalid input. Please enter a number.")
        except KeyboardInterrupt:
            print("\nExiting...")
            sys.exit(0)

def load_config(config_file='config.ini') -> configparser.ConfigParser:
    config = configparser.ConfigParser()
    
    config['GPIO'] = {
        'dir_pin': '20',
        'step_pin': '21',
        'steps_per_rotation': '200',
        'step_delay': '0.005'
    }
    
    config['MQTT'] = {
        'broker': '192.168.90.56',
        'port': '1883',
        'topic': 'rotary_encoder/data',
        'client_id': f'headland_manager_{int(time.time())}'
    }
    
    config['Thresholds'] = {
        'steering_threshold': '25',
        'gathering_upper_threshold': '15',
        'gathering_lower_threshold': '10',
        'hysteresis': '1',
        'high_threshold': '15',
        'low_threshold': '5',
        'casting_lower_threshold': '15',
        'casting_upper_threshold': '25'
    }
    
    try:
        config.read(config_file)
        logger.info(f"Configuration loaded from {config_file}")
    except Exception as e:
        logger.warning(f"Could not load configuration: {e}")
        logger.info("Using default configuration")
        
    return config

def parse_args():
    parser = argparse.ArgumentParser(description='Headland Management System')
    parser.add_argument('-c', '--config', default='config.ini', help='Path to config file')
    parser.add_argument('-d', '--debug', action='store_true', help='Enable debug logging')
    parser.add_argument('-s', '--simulate', action='store_true', help='Run in simulation mode (no hardware)')
    return parser.parse_args()

def main():
    args = parse_args()
    
    if args.debug:
        logger.setLevel(logging.DEBUG)
    
    try:
        config = load_config(args.config)
        
        motor = MotorController(
            dir_pin=config.getint('GPIO', 'dir_pin'),
            step_pin=config.getint('GPIO', 'step_pin'),
            steps_per_rotation=config.getint('GPIO', 'steps_per_rotation'),
            simulate=args.simulate
        )
        
        mqtt_handler = MQTTHandler(
            broker=config.get('MQTT', 'broker'),
            port=config.getint('MQTT', 'port'),
            topic=config.get('MQTT', 'topic')
        )
        
        mqtt_connected = mqtt_handler.connect()
        if not mqtt_connected:
            logger.warning("Starting with initial steering angle 0")
        
        manager = HeadlandManager(motor, mqtt_handler, config)
        
        pattern = select_algorithm()
        logger.info(f"Starting headland management with {HeadlandPattern.get_name(pattern)} algorithm")
        
        try:
            while True:
                manager.run_algorithm(pattern)
                time.sleep(0.1)
        except KeyboardInterrupt:
            logger.info("Process interrupted by user")
        
    except Exception as e:
        logger.error(f"Unexpected error: {e}", exc_info=True)
    finally:
        if 'mqtt_handler' in locals():
            mqtt_handler.disconnect()
        if 'motor' in locals():
            motor.cleanup()
        logger.info("Application shut down")

if _name_ == "_main_":
    main()
