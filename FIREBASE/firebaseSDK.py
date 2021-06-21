import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import json
import numpy as np

cred = credentials.Certificate('firebase_SDK.json')
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://rexinterface-default-rtdb.europe-west1.firebasedatabase.app/'
})

myDict = {}
  
# # Adding list as value
myDict["Points"] = []
myDict["Human_pose"] = []
print(myDict)

# 3D POINTS

A = ['5', '10', '15']
mat = np.array([])
mat = np.append(mat, A)

for a in range(20):
    mat = np.vstack([mat, A])

mat_tolist = mat.tolist()
myDict["Points"].extend(mat_tolist)
# print("MY DICT: ", myDict)



# HUMAN POSE

A = ['8', '10', '15', '20']
D = ['5', '10', '15', '20']
mat_human = np.array([])
mat_human = np.append(mat_human, A)
mat_human = np.vstack([mat_human, D])

mat_tolist_human = mat_human.tolist()

myDict["Human_pose"].extend(mat_tolist_human)

# myDict["Human_pose"][0] = ['5', '10']
print(myDict)



# msg = pickle.dumps(myDict)
# msg = bytes(f"{len(msg):<{HEADERSIZE}}", 'utf-8')+msg
# print("PICKLE DUMPS: ", msg)

# b = json.dumps(myDict)
# print(b)

ref = db.reference('/')
# ref.set({
#     '3D_env':
#     {
#         '1': '[10, 10, 200]',
#         '2': '[15, 13, 150]',
#         '3': '[15, 13, 150]',
#         '4': '[15, 13, 150]'
#     },
#     'Human_pose':
#     {
#         '1' : '[10, 10, 20, 20]',
#         '2': '[15, 15, 35, 35]'
#     }
# })
ref.set(myDict)



#region updating

# ref = db.reference('Human_pose')

# mydict = {}
# mydict["0"] = []
# mydict["1"] = []

# A = ['5', '10', '15', '20']
# mat = np.array([])
# mat = np.append(mat, A)

# mat_tolist = mat.tolist()
# mydict["0"].extend(mat_tolist)
# # mydict["1"].extend(mat_tolist)

# print("MY DICT: ", mydict)

# ref.update(mydict)

#endregion

#region adding value using push

# A = ['5', '10', '15', '20']
# mat = np.array([])
# mat = np.append(mat, A)

# mat_tolist = mat.tolist()
# mydict["2"].extend(mat_tolist)
# mydict["3"].extend(mat_tolist)

# print("MY DICT: ", mydict)

# ref = db.reference('Human_pose')
# emp_ref = ref.push(mydict)

#endregion  