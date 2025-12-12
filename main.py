import asyncio
import atexit
import json
import logging
import signal
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional
import yaml
import dht11
import paho.mqtt.client as mqtt
from telegram import Update
from telegram.ext import (
    Application,
    CommandHandler,
    ContextTypes,
    MessageHandler,
    filters,
)
from time import sleep

import RPi.GPIO as GPIO


# =============================================================================
# LOGGING SETUP
# =============================================================================

logging.basicConfig(
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    level=logging.INFO,
)
logger = logging.getLogger(__name__)

logging.getLogger("httpx").setLevel(logging.WARNING)
logging.getLogger("httpcore").setLevel(logging.WARNING)
logging.getLogger("telegram").setLevel(logging.WARNING)

# =============================================================================
# CONFIGURATION
# =============================================================================

class Config:
    def __init__(self, config_path: Optional[Path] = None):
        if config_path is None:
            script_dir = Path(__file__).parent.resolve()
            config_path = script_dir / "config.yaml"
        self.config_path = config_path
        self._load_config()
    
    def _load_config(self):
        if not self.config_path.exists():
            logger.error(f"Configuration file not found: {self.config_path}")
            sys.exit(1)

        try:
            with open(self.config_path, "r") as f:
                cfg = yaml.safe_load(f)
        except yaml.YAMLError as e:
            logger.error(f"Error parsing configuration file: {e}")
            sys.exit(1)
        except Exception as e:
            logger.error(f"Error loading configuration file: {e}")
            sys.exit(1)

        telegram = cfg.get("telegram", {})
        self.telegram_bot_token: str = telegram.get("bot_token", "")
        self.allowed_user_ids: list[int] = telegram.get("allowed_user_ids", [])

        if not self.telegram_bot_token or self.telegram_bot_token == "BOT_TOKEN":
            logger.error("Please set a valid Telegram bot token in config.yaml")
            sys.exit(1)

        gpio = cfg.get("gpio", {})
        self.dht_pin = int(gpio.get('dht_pin', 4))
        self.relay_pin = int(gpio.get('relay_pin', 17))
        self.relay_active_low = bool(gpio.get('relay_active_low', True))

        mqtt_cfg = cfg.get('mqtt', {})
        self.mqtt_enabled = bool(mqtt_cfg.get('enabled', True))
        self.mqtt_broker = str(mqtt_cfg.get('broker', 'localhost'))
        self.mqtt_port = int(mqtt_cfg.get('port', 1883))
        self.mqtt_reconnect_interval = int(mqtt_cfg.get('reconnect_interval', 30))
        self.mqtt_topic = str(mqtt_cfg.get('topic', 'thermostat'))
        self.mqtt_metric_prefix = str(mqtt_cfg.get('metric_prefix', 'thermostat'))
        
        control = cfg.get('control', {})
        self.measurement_interval = int(control.get('measurement_interval', 60))
        self.temperature_hysteresis = float(control.get('temperature_hysteresis', 0.5))
        self.min_temperature = float(control.get('min_temperature', 10))
        self.max_temperature = float(control.get('max_temperature', 23))
        
        logger.info(f"Configuration loaded from {self.config_path}")

cfg = Config()

# =============================================================================
# Thermostat State
# =============================================================================

class ThermostatState:
    def __init__(self):
        self.temperature: Optional[float] = None
        self.humidity: Optional[float] = None
        self.target_temperature: Optional[float] = None
        self.heating_on: bool = False
        self.control_enabled: bool = False
        self.last_measurement: Optional[datetime] = None
        self.mqtt_connected: bool = False
        self.sensor_error: Optional[str] = None

state = ThermostatState()
mqtt_client: Optional[mqtt.Client] = None
dht_sensor = None

# =============================================================================
# GPIO MANAGEMENT
# =============================================================================

def setup_gpio():
    """Initialize GPIO pins."""
    if GPIO is None:
        logger.warning("GPIO not available - skipping GPIO setup")
        return
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(cfg.relay_pin, GPIO.OUT)
    turn_off_relay()
    logger.info(f"GPIO initialized - Relay pin: {cfg.relay_pin}")


def turn_on_relay():
    """Turn the relay ON (heating ON)."""
    if GPIO is None:
        logger.info("SIMULATION: Relay ON")
        return
    
    GPIO.output(cfg.relay_pin, GPIO.HIGH)
    logger.info("Relay turned ON")


def turn_off_relay():
    """Turn the relay OFF (heating OFF)."""
    if GPIO is None:
        logger.info("SIMULATION: Relay OFF")
        return
    
    GPIO.output(cfg.relay_pin, GPIO.LOW)
    logger.info("Relay turned OFF")


def cleanup_gpio():
    """
    Cleanup GPIO - Always turn off relay on exit.
    """
    logger.warning("Cleaning up GPIO - turning off relay for safety")
    try:
        turn_off_relay()
        if GPIO is not None:
            GPIO.cleanup()
    except Exception as e:
        logger.error(f"Error during GPIO cleanup: {e}")
        # Try to force the pin low
        if GPIO is not None:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(cfg.relay_pin, GPIO.OUT)
                GPIO.output(cfg.relay_pin, GPIO.LOW)
                GPIO.cleanup()
            except:
                pass

# =============================================================================
# DHT SENSOR
# =============================================================================

def setup_sensor():
    """Initialize the DHT11 sensor."""
    global dht_sensor
    if GPIO is None:
        logger.warning("GPIO not available - sensor will not work")
        dht_sensor = None
        return
    
    try:
        dht_sensor = dht11.DHT11(pin=cfg.dht_pin)
        logger.info(f"DHT11 sensor initialized on GPIO{cfg.dht_pin}")
        
        sleep(2)
        
        for i in range(3):
            result = dht_sensor.read()
            if result.is_valid():
                logger.info(f"Sensor ready: {result.temperature}°C, {result.humidity}%")
                state.temperature = float(result.temperature)
                state.humidity = float(result.humidity)
                state.sensor_error = None
                return
            sleep(2)
        
        logger.warning("Sensor initialized but initial readings invalid (will retry in main loop)")
        
    except Exception as e:
        logger.error(f"Failed to initialize DHT11 sensor: {e}")
        dht_sensor = None


def read_sensor() -> tuple[Optional[float], Optional[float]]:
    """
    Read temperature and humidity from the DHT11 sensor.
    Returns (temperature, humidity) or (None, None) on error.
    """
    global dht_sensor
    
    if dht_sensor is None:
        state.sensor_error = "Sensor not initialized"
        return None, None
    
    try:
        result = dht_sensor.read()
        
        if result.is_valid():
            temperature = float(result.temperature)
            humidity = float(result.humidity)
            state.sensor_error = None
            return temperature, humidity
        else:
            state.sensor_error = f"Invalid reading (error code: {result.error_code})"
            logger.debug(f"Sensor read invalid: error code {result.error_code}")
            return None, None
            
    except Exception as e:
        state.sensor_error = str(e)
        logger.error(f"Unexpected sensor error: {e}")
        return None, None

# =============================================================================
# MQTT CLIENT
# =============================================================================

def on_mqtt_connect(client, userdata, flags, rc, properties=None):
    """Callback when MQTT connects."""
    if rc == 0:
        state.mqtt_connected = True
        logger.info("MQTT connected successfully")
    else:
        state.mqtt_connected = False
        logger.warning(f"MQTT connection failed with code: {rc}")


def on_mqtt_disconnect(client, userdata, rc, properties=None):
    """Callback when MQTT disconnects."""
    state.mqtt_connected = False
    logger.warning(f"MQTT disconnected (rc={rc})")


def setup_mqtt() -> Optional[mqtt.Client]:
    """Initialize MQTT client (non-blocking, tolerates failures)."""
    if not cfg.mqtt_enabled:
        logger.info("MQTT disabled in configuration")
        return None
    
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        client.on_connect = on_mqtt_connect
        client.on_disconnect = on_mqtt_disconnect
        
        client.reconnect_delay_set(min_delay=1, max_delay=cfg.mqtt_reconnect_interval)
        
        return client
    except Exception as e:
        logger.error(f"Failed to create MQTT client: {e}")
        return None


async def connect_mqtt():
    """Attempt to connect to MQTT broker."""
    global mqtt_client
    
    if mqtt_client is None or not cfg.mqtt_enabled:
        return
    
    if state.mqtt_connected:
        return
    
    try:
        mqtt_client.connect_async(cfg.mqtt_broker, cfg.mqtt_port, keepalive=60)
        mqtt_client.loop_start()
        logger.info(f"MQTT connection initiated to {cfg.mqtt_broker}:{cfg.mqtt_port}")
    except Exception as e:
        logger.warning(f"MQTT connection attempt failed: {e}")
        state.mqtt_connected = False


def publish_mqtt_metrics():
    """Publish all metrics as JSON to single MQTT topic."""
    if mqtt_client is None or not cfg.mqtt_enabled:
        return
    
    if not state.mqtt_connected:
        logger.debug("MQTT not connected, skipping publish")
        return
    
    prefix = cfg.mqtt_metric_prefix
    
    # Build metrics dict for mqtt2prom
    # Each key becomes a separate metric name
    metrics = {}
    
    if state.temperature is not None:
        metrics[f"{prefix}_temperature"] = state.temperature
    
    if state.humidity is not None:
        metrics[f"{prefix}_humidity"] = state.humidity
    
    # Heating status as 0/1 for gauge
    metrics[f"{prefix}_heating"] = 1 if state.heating_on else 0
    
    # Target temperature (0 if not set/disabled)
    if state.control_enabled and state.target_temperature is not None:
        metrics[f"{prefix}_target"] = state.target_temperature
    else:
        metrics[f"{prefix}_target"] = 0
    
    try:
        payload = json.dumps(metrics)
        mqtt_client.publish(cfg.mqtt_topic, payload, qos=1)
        logger.debug(f"MQTT published: {cfg.mqtt_topic} = {payload}")
    except Exception as e:
        logger.warning(f"MQTT publish failed: {e}")

# =============================================================================
# THERMOSTAT CONTROL
# =============================================================================

def update_heating_state():
    """
    Decide whether heating should be on or off based on current temperature
    and target temperature. Uses hysteresis to prevent rapid cycling.
    """
    if not state.control_enabled or state.target_temperature is None:
        if state.heating_on:
            state.heating_on = False
            turn_off_relay()
        return
    
    if state.temperature is None:
        if state.heating_on:
            logger.warning("No temperature reading - turning off heating for safety")
            state.heating_on = False
            turn_off_relay()
        return
    
    lower_threshold = state.target_temperature - cfg.temperature_hysteresis
    upper_threshold = state.target_temperature + cfg.temperature_hysteresis
    
    if state.temperature < lower_threshold and not state.heating_on:
        state.heating_on = True
        turn_on_relay()
        logger.info(f"Heating ON (temp {state.temperature}°C < {lower_threshold}°C)")
    elif state.temperature > upper_threshold and state.heating_on:
        state.heating_on = False
        turn_off_relay()
        logger.info(f"Heating OFF (temp {state.temperature}°C > {upper_threshold}°C)")


async def measurement_loop():
    """
    Main measurement loop - runs every MEASUREMENT_INTERVAL seconds.
    Reads sensor, publishes to MQTT, updates heating control.
    """
    while True:
        try:            
            if cfg.mqtt_enabled and not state.mqtt_connected:
                await connect_mqtt()
            
            temp, humidity = read_sensor()
            
            if temp is not None and humidity is not None:
                state.temperature = temp
                state.humidity = humidity
                state.last_measurement = datetime.now()
                
                logger.info(f"Measurement: {temp:.1f}°C, {humidity:.1f}%")
            
            update_heating_state()
            
            # Publish all metrics as single JSON message
            publish_mqtt_metrics()
            
        except Exception as e:
            logger.error(f"Error in measurement loop: {e}")
            # On any error, ensure heating is off for safety
            try:
                turn_off_relay()
                state.heating_on = False
            except:
                pass
        
        await asyncio.sleep(cfg.measurement_interval)

# =============================================================================
# TELEGRAM BOT
# =============================================================================

def is_authorized(user_id: int) -> bool:
    """Check if a user is authorized to use the bot."""
    return user_id in cfg.allowed_user_ids


async def check_auth(update: Update) -> bool:
    """Check authorization and send rejection message if unauthorized."""
    user_id = update.effective_user.id
    if not is_authorized(user_id):
        logger.warning(f"Unauthorized access attempt from user {user_id}")
        return False
    return True


async def cmd_start(update: Update, context: ContextTypes.DEFAULT_TYPE):
    if not await check_auth(update):
        return
    await cmd_help(update, context)


async def cmd_help(update: Update, context: ContextTypes.DEFAULT_TYPE):
    if not await check_auth(update):
        return
    
    help_text = """*Thermostat Bot Commands*

/help - Show this help message
/status - Display current readings and settings
/temp X - Set target temperature to X°C and enable control
/off - Turn off temperature control

*Examples:*
• `/temp 21` - Set target to 21°C
• `/temp 19.5` - Set target to 19.5°C
• `/off` - Disable heating control and turn off heating
"""
    await update.message.reply_text(help_text, parse_mode="Markdown")

async def make_status():
    read_sensor()
    # Temperature reading
    if state.temperature is not None:
        temp_str = f"{state.temperature:.1f}°C"
    else:
        temp_str = "N/A"
    
    # Humidity reading
    if state.humidity is not None:
        humidity_str = f"{state.humidity:.1f}%"
    else:
        humidity_str = "N/A"
    
    # Target temperature
    if state.control_enabled and state.target_temperature is not None:
        target_str = f"{state.target_temperature:.1f}°C"
    else:
        target_str = "Not set"
    
    # Control status
    if state.control_enabled:
        control_str = "✅ Enabled"
    else:
        control_str = "❌ Disabled"
    
    # Heating status
    if state.heating_on:
        heating_str = "🔥 ON"
    else:
        heating_str = "❄️ OFF"
    
    # MQTT status
    if cfg.mqtt_enabled:
        mqtt_str = "✅ Connected" if state.mqtt_connected else "❌ Disconnected"
    else:
        mqtt_str = "Disabled"
    
    # Last measurement time
    if state.last_measurement:
        time_str = state.last_measurement.strftime("%H:%M:%S")
    else:
        time_str = "Never"
    
    status_text = f"""📊 *Thermostat Status*

🌡️ Temperature: {temp_str}
💧 Humidity: {humidity_str}
🎯 Target: {target_str}

🔧 Control: {control_str}
🔥 Heating: {heating_str}

📡 MQTT: {mqtt_str}
🕐 Last reading: {time_str}
"""
    
    return status_text

async def cmd_status(update: Update, context: ContextTypes.DEFAULT_TYPE):
    if not await check_auth(update):
        return
    
    status_text = await make_status()
    await update.message.reply_text(status_text, parse_mode="Markdown")


async def cmd_temp(update: Update, context: ContextTypes.DEFAULT_TYPE):
    """Handle /temp X command - set target temperature."""
    if not await check_auth(update):
        return
    
    if not context.args:
        await update.message.reply_text(
            "❌ Please specify a temperature.\nExample: `/temp 21`",
            parse_mode="Markdown"
        )
        return
    
    try:
        target = float(context.args[0])
        
        if target < cfg.min_temperature or target > cfg.max_temperature:
            await update.message.reply_text(
                f"❌ Temperature must be between {cfg.min_temperature}°C and {cfg.max_temperature}°C."
            )
            return
        
        state.target_temperature = target
        state.control_enabled = True
        
        update_heating_state()
        publish_mqtt_metrics()
        
        heating_status = "🔥 ON" if state.heating_on else "❄️ OFF"
        await update.message.reply_text(
            f"✅ Target temperature set to {target:.1f}°C\n"
            f"Temperature control: ✅ Enabled\n"
            f"Heating: {heating_status}"
        )
        logger.info(f"Target temperature set to {target}°C by user {update.effective_user.id}")
        
    except ValueError:
        await update.message.reply_text(
            "❌ Invalid temperature value.\nExample: `/temp 21` or `/temp 19.5`",
            parse_mode="Markdown"
        )


async def cmd_off(update: Update, context: ContextTypes.DEFAULT_TYPE):
    if not await check_auth(update):
        return
    
    state.control_enabled = False
    state.heating_on = False
    turn_off_relay()
    
    publish_mqtt_metrics()
    
    await update.message.reply_text(
        "✅ Temperature control disabled.\n"
        "❄️ Heating: OFF"
    )
    logger.info(f"Temperature control disabled by user {update.effective_user.id}")


async def handle_unauthorized_message(update: Update, context: ContextTypes.DEFAULT_TYPE):
    """Handle any message from unauthorized users."""
    user_id = update.effective_user.id
    if not is_authorized(user_id):
        logger.warning(f"Unauthorized message from user {user_id}")
        
# =============================================================================
# SIGNAL HANDLERS FOR SAFE SHUTDOWN
# =============================================================================

def signal_handler(signum, frame):
    """Handle shutdown signals - ensure relay is turned off."""
    logger.warning(f"Received signal {signum} - initiating safe shutdown")
    cleanup_gpio()
    sys.exit(0)

# =============================================================================
# MAIN
# =============================================================================

async def send_startup_message(application: Application):
    """Send a startup notification to all authorized users."""

    startup_text = (
        "🟢 *Thermostat Bot Started*\n\n"
        f"🕐 Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"    
        "Use /help for available commands.\nControl is currently disabled."
    )
    startup_text += "\n"
    startup_text += await make_status()
    
    for user_id in cfg.allowed_user_ids:
        try:
            await application.bot.send_message(
                chat_id=user_id,
                text=startup_text,
                parse_mode="Markdown"
            )
            logger.info(f"Startup message sent to user {user_id}")
        except Exception as e:
            logger.warning(f"Failed to send startup message to user {user_id}: {e}")

async def main():
    """Main entry point."""
    global mqtt_client
    
    logger.info("=" * 50)
    logger.info("Starting Thermostat Bot")
    logger.info("=" * 50)
    
    # Register cleanup handlers
    atexit.register(cleanup_gpio)
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize hardware
    setup_gpio()
    setup_sensor()
    
    # Initialize MQTT
    mqtt_client = setup_mqtt()
    if mqtt_client:
        await connect_mqtt()
    
    # Create Telegram bot application
    application = Application.builder().token(cfg.telegram_bot_token).build()
    
    # Add command handlers
    application.add_handler(CommandHandler("start", cmd_start))
    application.add_handler(CommandHandler("help", cmd_help))
    application.add_handler(CommandHandler("status", cmd_status))
    application.add_handler(CommandHandler("temp", cmd_temp))
    application.add_handler(CommandHandler("off", cmd_off))
    
    # Handle any other messages (for unauthorized user feedback)
    application.add_handler(
        MessageHandler(filters.ALL, handle_unauthorized_message)
    )
    
    # Start the measurement loop as a background task
    asyncio.create_task(measurement_loop())
    
    # Start the bot
    logger.info("Bot starting...")
    await application.initialize()
    await application.start()
    await application.updater.start_polling(drop_pending_updates=True)
    
    await send_startup_message(application)

    logger.info("Bot is running. Press Ctrl+C to stop.")
    
    # Keep running until interrupted
    try:
        while True:
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        pass
    finally:
        logger.info("Shutting down...")
        await application.updater.stop()
        await application.stop()
        await application.shutdown()
        
        # Stop MQTT
        if mqtt_client:
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
        
        cleanup_gpio()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Fatal error: {e}")
    finally:
        cleanup_gpio()