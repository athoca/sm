from smartdrone.vehicles import PLSmartDrone

handler = None
smartdrone = PLSmartDrone(handler)
smartdrone.start_main_control_loop()