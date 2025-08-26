import carla, time

client = carla.Client('winhost', 2000)
world = client.get_world()

settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.04
world.apply_settings(settings)

for actor in world.get_actors():
    if actor.attributes.get('role_name') == 'ego_vehicle':
        ego = actor
        break

traffic_manager = client.get_trafficmanager()
time.sleep(5)
ego.set_autopilot(True, traffic_manager.get_port())
print(ego)
for _ in range(500):
    world.tick()