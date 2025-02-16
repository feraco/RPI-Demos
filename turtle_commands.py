def run_commands(turtle_controller):
    """Define and execute movement commands using move_turtle()."""
    turtle_controller.move_turtle(2.0, 0.0, 2.0)  # Move forward for 2 seconds
    turtle_controller.move_turtle(0.0, 1.5, 1.5)  # Turn right for 1.5 seconds
    turtle_controller.move_turtle(1.0, 0.5, 3.0)  # Move in a curve for 3 seconds
