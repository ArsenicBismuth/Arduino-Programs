import requests
import yaml
from pathlib import Path

# Ref: https://www.espboards.dev/blog/send-whatsapp-message-from-esp32/
# Ref: https://developers.facebook.com/docs/whatsapp/cloud-api/guides/send-message-templates

print("========================================")
print("  WhatsApp API Test Script")
print("========================================")
print()

# Load secrets from parent directory
secrets_path = Path(__file__).parent.parent / "secrets.yaml"
with open(secrets_path, "r") as f:
    secrets = yaml.safe_load(f)

# If any of these are not set, it'll throw an error
try:
    access_token = secrets["wa_token"]
    phone_number = secrets["wa_phone_number"]
    app_url = secrets["wa_app_url"]
except KeyError as e:
    print(f"Error: Missing required secret: {e}")
    print("Please ensure ../secrets.yaml is present and contains the required values.")
    input("Press Enter to exit...")
    exit(1)

print(f"Phone Number: {phone_number}")
print(f"Token: {access_token[:20]}...")

print()
print("Sending WhatsApp message...")
print()

url = app_url

headers = {
    "Authorization": f"Bearer {access_token}",
    "Content-Type": "application/json"
}

json_body = {
    "messaging_product": "whatsapp",
    "to": phone_number,
    "type": "template",
    "template": {
        "name": "door_status_open",
        # "name": "door_status_closed",
        # "name": "door_warning_open",
        "language": {"code": "en"},
        "components": [
            {
                "type": "body",
                "parameters": [
                    {"type": "text", "parameter_name": "time_close", "text": "11:22"},
                    {"type": "text", "parameter_name": "time_open", "text": "19:46"}
                    # {"type": "text", "parameter_name": "time", "text": "19:46"}
                ]
            }
        ]
    }
}

response = requests.post(url, headers=headers, json=json_body)

print(f"Status Code: {response.status_code}")
print()
print("Response Headers:")
for key, value in response.headers.items():
    print(f"  {key}: {value}")
print()
print("Response Body:")
print(response.text)

print()
print("========================================")
print("  Request complete.")
print("========================================")
input("Press Enter to exit...")

