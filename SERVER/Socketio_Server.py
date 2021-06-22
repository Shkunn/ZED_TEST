import socketio

# create a Socket.IO server
sio = socketio.Server()

# wrap with a WSGI application
app = socketio.WSGIApp(sio)

@sio.event
def my_event(sid, data):
    pass

@sio.on('my custom event')
def another_event(sid, data):
    pass

@sio.event
def connect(sid, environ, auth):
    print('connect ', sid)

@sio.event
def disconnect(sid):
    print('disconnect ', sid)