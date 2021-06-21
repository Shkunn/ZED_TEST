from firebase import firebase

firebase = firebase.FirebaseApplication("https://rexinterface-default-rtdb.europe-west1.firebasedatabase.app/", None)

firebase.delete('/rexinterface-default-rtdb/Points/', '-MchjOqXVWyV22thQ96g')
print('Record Deleted')