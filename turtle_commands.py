def run_commands(turtle_controller):
    """Define and execute movement commands."""
    # Example movement commands
    turtle_controller.send_command(0.0, 0.0, 2.0)  # Move forward for 2 seconds
    turtle_controller.send_command(0.0, 1.0, 1.5)  # Turn right for 1.5 seconds
    turtle_controller.send_command(1.0, 0.5, 3.0)  # Move in a curve for 3 seconds
