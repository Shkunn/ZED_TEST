from firebase import firebase 


firebase = firebase.FirebaseApplication("https://rexinterface-default-rtdb.europe-west1.firebasedatabase.app/")

result = firebase.put('/rexinterface-default-rtdb/Customer/-MchjOqXVWyV22thQ96g', 'Name', 'Polpi')
print(result)