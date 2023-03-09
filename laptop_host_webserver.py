from flask import Flask, request
from flask_socketio import SocketIO

app = Flask(__name__)
socketio = SocketIO(app)

@app.route('/')
def home():
    return '''
        <html>
            <body>
                <h1>BlueTech Teleop Controls</h1>
                <p>Click on one of the buttons below to move either side of the robot.</p>
                <form>
                    <button type="button" id="right">Move Right</button>
                    <button type="button" id="left">Move Left</button>
                    <button type="button" id="stop">Stop ALL</button>

                </form>
                <div id="response"></div>
                <script>
                    const buttons = document.querySelectorAll('button');
                    buttons.forEach(button => {
                        button.addEventListener('click', event => {
                            const string = event.target.id;
                            const xhr = new XMLHttpRequest();
                            xhr.open('POST', '/send', true);
                            xhr.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded; charset=UTF-8');
                            xhr.onload = () => {
                                const response = document.getElementById('response');
                                response.innerHTML = xhr.responseText;
                            };
                            xhr.send(`string=${string}`);
                        });
                    });
                </script>
            </body>
        </html>
    '''

@app.route('/send', methods=['POST'])
def send():
    string = request.form['string']
    socketio.emit('string', string)
    return f'Motor Status: {string}'

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000)
