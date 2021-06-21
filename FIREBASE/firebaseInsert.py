from firebase import firebase 
import numpy as np
import json

# Creating empty dictionaries
human_dict = {}
env_points = {}


human_json = json.dumps(human_dict)
env_json = json.dumps(env_points)


firebase = firebase.FirebaseApplication("https://rexinterface-default-rtdb.europe-west1.firebasedatabase.app/")


result = firebase.post('/rexinterface-default-rtdb/3D_points', human_json)

result = firebase.post('/rexinterface-default-rtdb/Human_points', env_points)

print(result)