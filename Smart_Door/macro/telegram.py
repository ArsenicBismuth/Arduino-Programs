#!/usr/bin/env python3
"""
Telegram Bot Message Sender
Sends a message to a specific user using the Telegram Bot API (raw HTTP requests).
Configuration is read from secrets.yaml in the same directory.
"""

import urllib.request
import urllib.parse
import json
import sys
from pathlib import Path

import yaml


def load_secrets(secrets_path: Path) -> dict:
    """Load secrets from a YAML file."""
    with open(secrets_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def telegram_api_request(bot_token: str, method: str, payload: dict = None) -> dict:
    """
    Make a request to the Telegram Bot API.
    
    Args:
        bot_token: Telegram bot token
        method: API method (e.g., 'sendMessage', 'getUpdates')
        payload: Optional JSON payload
        
    Returns:
        API response as dictionary
    """
    url = f"https://api.telegram.org/bot{bot_token}/{method}"
    
    if payload:
        data = json.dumps(payload).encode('utf-8')
    else:
        data = None
    
    request = urllib.request.Request(
        url,
        data=data,
        headers={
            "Content-Type": "application/json",
            "Accept": "application/json"
        },
        method="POST" if data else "GET"
    )
    
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            result = json.loads(response.read().decode('utf-8'))
            return result
    except urllib.error.HTTPError as e:
        error_body = e.read().decode('utf-8')
        return {"ok": False, "error": f"HTTP {e.code}", "description": error_body}
    except urllib.error.URLError as e:
        return {"ok": False, "error": "Network error", "description": str(e.reason)}


def get_updates(bot_token: str) -> dict:
    """Get recent messages/updates sent to the bot."""
    return telegram_api_request(bot_token, "getUpdates")


def send_message(bot_token: str, chat_id: str, message: str) -> dict:
    """Send a message to a Telegram user."""
    payload = {
        "chat_id": chat_id,
        "text": message,
        "parse_mode": "HTML"
    }
    return telegram_api_request(bot_token, "sendMessage", payload)


def print_chat_ids(bot_token: str):
    """Fetch and display chat IDs from recent messages."""
    print("Fetching recent updates...")
    print("(Make sure you've sent a message to your bot first)\n")
    
    result = get_updates(bot_token)
    
    if not result.get("ok"):
        print("✗ Failed to get updates")
        print(f"  Error: {result.get('error', 'Unknown')}")
        print(f"  Details: {result.get('description', 'No details')}")
        return
    
    updates = result.get("result", [])
    
    if not updates:
        print("No recent messages found.")
        print("Send a message to your bot on Telegram, then run this again.")
        return
    
    # Collect unique chats
    chats = {}
    for update in updates:
        msg = update.get("message") or update.get("edited_message") or {}
        chat = msg.get("chat", {})
        chat_id = chat.get("id")
        if chat_id and chat_id not in chats:
            chats[chat_id] = {
                "id": chat_id,
                "type": chat.get("type", "unknown"),
                "name": chat.get("first_name", "") + " " + chat.get("last_name", ""),
                "username": chat.get("username", ""),
            }
    
    print(f"Found {len(chats)} unique chat(s):\n")
    print("-" * 50)
    for chat_id, info in chats.items():
        print(f"  Chat ID: {chat_id}")
        print(f"  Type:    {info['type']}")
        if info['name'].strip():
            print(f"  Name:    {info['name'].strip()}")
        if info['username']:
            print(f"  Username: @{info['username']}")
        print("-" * 50)


def send_message_interactive(bot_token: str, main_chat_id: str):
    """Send a message interactively."""
    if not main_chat_id or main_chat_id == "some_telegram_main_chat_id":
        print("Error: telegram_main_chat_id not configured in secrets.yaml")
        print("Use option 1 to get your chat ID first.")
        return
    
    message = input("Enter message (or press Enter for default): ").strip()
    if not message:
        message = "Hello from Smart Door! 🚪"
    
    print(f"\nSending message to user {main_chat_id}...")
    result = send_message(bot_token, main_chat_id, message)
    
    if result.get("ok"):
        print("✓ Message sent successfully!")
        msg_id = result.get("result", {}).get("message_id")
        if msg_id:
            print(f"  Message ID: {msg_id}")
    else:
        print("✗ Failed to send message")
        print(f"  Error: {result.get('error', 'Unknown')}")
        print(f"  Details: {result.get('description', 'No details')}")


def read_latest_message(bot_token: str, filter_chat_id: str = None):
    """
    Fetch and display the latest message received by the bot.
    
    Args:
        bot_token: Telegram bot token
        filter_chat_id: If provided, only show messages from this chat ID
    """
    if filter_chat_id:
        print(f"Fetching latest message from chat {filter_chat_id}...\n")
    else:
        print("Fetching latest message...\n")
    
    result = get_updates(bot_token)
    
    if not result.get("ok"):
        print("✗ Failed to get updates")
        print(f"  Error: {result.get('error', 'Unknown')}")
        print(f"  Details: {result.get('description', 'No details')}")
        return
    
    updates = result.get("result", [])
    
    if not updates:
        print("No messages found.")
        print("Send a message to your bot on Telegram first.")
        return
    
    # Find the latest message (optionally filtered by chat_id)
    latest_msg = None
    for update in reversed(updates):
        msg = update.get("message") or update.get("edited_message") or {}
        if not msg:
            continue
        chat = msg.get("chat", {})
        chat_id = str(chat.get("id", ""))
        
        if filter_chat_id:
            if chat_id == str(filter_chat_id):
                latest_msg = msg
                break
        else:
            latest_msg = msg
            break
    
    if not latest_msg:
        if filter_chat_id:
            print(f"No messages found from chat {filter_chat_id}.")
        else:
            print("No message content in updates.")
        return
    
    chat = latest_msg.get("chat", {})
    sender = latest_msg.get("from", {})
    text = latest_msg.get("text", "(no text)")
    date = latest_msg.get("date", 0)
    
    # Convert timestamp to readable format
    from datetime import datetime
    date_str = datetime.fromtimestamp(date).strftime("%Y-%m-%d %H:%M:%S") if date else "Unknown"
    
    print("-" * 50)
    if filter_chat_id:
        # Just print message
        print(f"  Message:   {text}")
    else:
        print(f"  From:      {sender.get('first_name', '')} {sender.get('last_name', '')}".strip())
        if sender.get('username'):
            print(f"  Username:  @{sender.get('username')}")
        print(f"  Chat ID:   {chat.get('id')}")
        print(f"  Date:      {date_str}")
        print(f"  Message:   {text}")
    print("-" * 50)


def main():
    # Determine secrets file path (same directory as script)
    script_dir = Path(__file__).parent.parent
    secrets_path = script_dir / "secrets.yaml"
    
    if not secrets_path.exists():
        print(f"Error: secrets.yaml not found at {secrets_path}")
        print("Create it based on secrets.example.yaml")
        sys.exit(1)
    
    # Load configuration
    secrets = load_secrets(secrets_path)
    
    bot_token = secrets.get("telegram_bot_token")
    main_chat_id = secrets.get("telegram_chat_id")
    
    if not bot_token or bot_token == "some_telegram_bot_token":
        print("Error: telegram_bot_token not configured in secrets.yaml")
        sys.exit(1)
    
    # Check for command line arguments for non-interactive mode
    if len(sys.argv) > 1:
        if sys.argv[1] == "--get-id":
            print_chat_ids(bot_token)
        elif sys.argv[1] == "--send":
            message = " ".join(sys.argv[2:]) if len(sys.argv) > 2 else "Hello from Smart Door! 🚪"
            if not main_chat_id or main_chat_id == "some_telegram_main_chat_id":
                print("Error: telegram_main_chat_id not configured")
                sys.exit(1)
            result = send_message(bot_token, main_chat_id, message)
            if result.get("ok"):
                print("✓ Message sent!")
            else:
                print(f"✗ Failed: {result.get('description', 'Unknown error')}")
                sys.exit(1)
        elif sys.argv[1] == "--latest":
            read_latest_message(bot_token)
        elif sys.argv[1] == "--latest-main":
            if main_chat_id and main_chat_id != "some_telegram_main_chat_id":
                read_latest_message(bot_token, main_chat_id)
            else:
                print("telegram_chat_id not configured, reading any latest message...")
                read_latest_message(bot_token)
        else:
            print("Usage:")
            print("  python telegram_send.py                 # Interactive menu")
            print("  python telegram_send.py --get-id        # Get chat IDs")
            print("  python telegram_send.py --send [msg]    # Send message")
            print("  python telegram_send.py --latest        # Read latest message (any)")
            print("  python telegram_send.py --latest-main   # Read latest from main chat")
        return
    
    # Interactive menu
    print("\n=== Telegram Bot Utility ===\n")
    print("1. Get Chat IDs (from recent messages)")
    print("2. Send Message")
    print("3. Read Latest Message (any)")
    print("4. Read Latest Message (from main chat)")
    print("0. Exit")
    print()
    
    choice = input("Select option (0-4): ").strip()
    print()
    
    if choice == "0":
        print("Bye!")
    elif choice == "1":
        print_chat_ids(bot_token)
    elif choice == "2":
        send_message_interactive(bot_token, main_chat_id)
    elif choice == "3":
        read_latest_message(bot_token)
    elif choice == "4":
        if main_chat_id and main_chat_id != "some_telegram_main_chat_id":
            read_latest_message(bot_token, main_chat_id)
        else:
            print("telegram_chat_id not configured, reading any latest message...")
            read_latest_message(bot_token)
    else:
        print("Invalid option.")


if __name__ == "__main__":
    main()

