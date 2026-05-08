import airsim

# Connect to the simulation
client = airsim.MultirotorClient()
client.confirmConnection()

# Get a list of ALL objects in the scene using a wildcard regex ".*"
all_objects = client.simListSceneObjects(".*")
# all_objects = client.simListSceneObjects(".*[Cc]ube.*")

# print( client.simGetSegmentationObjectID("Cube"))
# Print them out alphabetically so you can read them
for obj in sorted(all_objects):
    print(obj)