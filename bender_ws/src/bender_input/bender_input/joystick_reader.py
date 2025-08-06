from inputs import get_gamepad

class JoystickReader:
    """
    Generic class to read joystick events using the `inputs` library.

    Attributes:
        thumbsticks (dict): Current state of thumbsticks (left_x, left_y, right_x, right_y)
        buttons (dict): Current state of buttons (pressed: True, released: False)
    """

    def __init__(self):
        # Map axis and button codes to logical names
        self.thumbsticks = {
            'left_x': 0,
            'left_y': 0,
            'right_x': 0,
            'right_y': 0
        }
        self.buttons = {}

        self.axis_map = {
            'ABS_X': 'left_x',
            'ABS_Y': 'left_y',
            'ABS_RX': 'right_x',
            'ABS_RY': 'right_y'
        }

    def poll(self) -> bool:
        """
        Poll for events and update thumbstick/button states.
        Returns True if any event was processed, otherwise False.
        """
        events = get_gamepad()
        changed = False

        for event in events:
            # Thumbsticks
            if event.ev_type == 'Absolute' and event.code in self.axis_map:
                self.thumbsticks[self.axis_map[event.code]] = self._normalize_axis(event.state)
                changed = True

            # Buttons
            elif event.ev_type == 'Key':
                self.buttons[event.code] = bool(event.state)
                changed = True

        return changed

    def get_thumbsticks(self) -> dict[str, float]:
        """
        Returns the current state of thumbsticks as a dict.
        """
        return self.thumbsticks.copy()

    def get_buttons(self) -> dict[str, bool]:
        """
        Returns the current state of buttons as a dict.
        """
        return self.buttons.copy()
    
    def _normalize_axis(self, raw_axis: int) -> float:
        """
        Normalizes the signed 16-bit gamepad input to a float from [-1.0, 1.0]
        """
        return raw_axis / 32767