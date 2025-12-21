# UNUSED

import requests
import yaml
from pathlib import Path

# Ref: https://developers.facebook.com/docs/whatsapp/phone-numbers/registration
# Ref: https://developers.facebook.com/docs/whatsapp/business-management-api/registering-phone-numbers

print("========================================")
print("  WhatsApp Phone Number Registration")
print("========================================")
print()

# Load secrets from parent directory
secrets_path = Path(__file__).parent.parent / "secrets.yaml"
with open(secrets_path, "r") as f:
    secrets = yaml.safe_load(f)

# Required secrets
try:
    access_token = secrets["wa_token"]
    # Extract phone number ID from the app URL (e.g., from .../896037763597183/messages)
    app_url = secrets["wa_app_url"]
    phone_number_id = app_url.split("/")[-2]
except KeyError as e:
    print(f"Error: Missing required secret: {e}")
    print("Please ensure ../secrets.yaml is present and contains the required values.")
    input("Press Enter to exit...")
    exit(1)

# API base URL
API_VERSION = "v24.0"
BASE_URL = f"https://graph.facebook.com/{API_VERSION}"

# Headers for all requests
headers = {
    "Authorization": f"Bearer {access_token}",
    "Content-Type": "application/json"
}

print(f"Phone Number ID: {phone_number_id}")
print(f"Token: {access_token[:20]}...")
print()

def register_phone_number(pin):
    """
    Register the phone number with the received verification code (PIN).
    
    Args:
        pin: The 6-digit verification code received via SMS or voice
    
    Returns:
        True if successful, False otherwise
    """
    print(f"Registering phone number with PIN: {pin}...")
    print()
    
    url = f"{BASE_URL}/{phone_number_id}/register"
    
    data = {
        "messaging_product": "whatsapp",
        "pin": str(pin)
    }
    
    response = requests.post(url, headers=headers, json=data)
    
    print(f"Status Code: {response.status_code}")
    print(f"Response: {response.text}")
    print()
    
    if response.status_code == 200:
        result = response.json()
        if result.get("success"):
            print("✓ Phone number registered successfully!")
            return True
    
    print("✗ Failed to register phone number.")
    return False


def get_phone_number_info():
    """
    Get information about the registered phone number.
    
    Returns:
        Phone number info dict or None
    """
    print("Fetching phone number info...")
    print()
    
    url = f"{BASE_URL}/{phone_number_id}"
    params = {
        "fields": "display_phone_number,verified_name,code_verification_status,quality_rating,platform_type,throughput,is_official_business_account"
    }
    
    response = requests.get(url, headers=headers, params=params)
    
    print(f"Status Code: {response.status_code}")
    print()
    
    if response.status_code == 200:
        result = response.json()
        print("Phone Number Info:")
        for key, value in result.items():
            print(f"  {key}: {value}")
        return result
    
    print(f"Response: {response.text}")
    return None


def deregister_phone_number():
    """
    Deregister the phone number from WhatsApp Business API.
    This is useful if you want to use the number elsewhere.
    
    Returns:
        True if successful, False otherwise
    """
    print("Deregistering phone number...")
    print()
    
    url = f"{BASE_URL}/{phone_number_id}/deregister"
    
    response = requests.post(url, headers=headers)
    
    print(f"Status Code: {response.status_code}")
    print(f"Response: {response.text}")
    print()
    
    if response.status_code == 200:
        result = response.json()
        if result.get("success"):
            print("✓ Phone number deregistered successfully!")
            return True
    
    print("✗ Failed to deregister phone number.")
    return False


def main():
    print("Select an action:")
    print("  3. Register phone number (enter PIN)")
    print("  4. Get phone number info")
    print("  5. Deregister phone number")
    print("  0. Exit")
    print("More info: https://developers.facebook.com/documentation/business-messaging/whatsapp/business-phone-numbers/registration")
    print()
    
    while True:
        choice = input("Enter your choice (0-5): ").strip()
        
        if choice == "0":
            print("Exiting...")
            break
        elif choice == "3":
            pin = input("Enter the 6-digit verification PIN: ").strip()
            if len(pin) == 6 and pin.isdigit():
                register_phone_number(pin)
            else:
                print("Invalid PIN. Please enter a 6-digit number.")
        elif choice == "4":
            get_phone_number_info()
        elif choice == "5":
            confirm = input("Are you sure you want to deregister? (yes/no): ").strip().lower()
            if confirm == "yes":
                deregister_phone_number()
            else:
                print("Deregistration cancelled.")
        else:
            print("Invalid choice. Please try again.")
        
        print()
        print("----------------------------------------")
        print()


if __name__ == "__main__":
    main()
    print()
    print("========================================")
    print("  Script complete.")
    print("========================================")
    input("Press Enter to exit...")

