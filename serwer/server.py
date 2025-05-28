from flask import Flask, request

app = Flask(__name__)

@app.route('/upload', methods=['POST'])
def upload_data():
    data = request.get_json()
    print(f"Received data from Bot {data['bot_id']}: {data['value']:.2f}")
    return {"status": "success"}, 200

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)  # Run server on localhost:5000
