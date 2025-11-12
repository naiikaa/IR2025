import time, pathlib, carla

xodr_path = pathlib.Path("kassel.xodr").resolve()
client = carla.Client("127.0.0.1", 2000)
client.set_timeout(30.0)

with open(xodr_path, "r", encoding="utf-8") as f:
    xodr = f.read()

# Create world from OpenDRIVE
world = client.generate_opendrive_world(xodr)

# Adjust settings safely
settings = world.get_settings()
settings.synchronous_mode = False
settings.no_rendering_mode = False
settings.fixed_delta_seconds = 0.0  # use variable step
world.apply_settings(settings)

print("[Dahanesho] Finaly OpenDRIVE world loaded:", xodr_path)
time.sleep(2)
