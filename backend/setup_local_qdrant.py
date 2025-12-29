"""
Setup script to configure a local Qdrant instance for development.

This script will:
1. Check if Docker is available
2. Run a local Qdrant container
3. Update your settings to use the local instance
"""

import os
import subprocess
import sys
import requests
import time

def check_docker():
    """Check if Docker is installed and running."""
    try:
        result = subprocess.run(['docker', '--version'], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            print("✅ Docker is installed")
            return True
        else:
            print("❌ Docker is not installed or not in PATH")
            return False
    except FileNotFoundError:
        print("❌ Docker is not installed or not in PATH")
        return False

def start_local_qdrant():
    """Start a local Qdrant instance using Docker."""
    print("Attempting to start local Qdrant instance...")
    
    # Stop any existing qdrant container
    subprocess.run(['docker', 'stop', 'qdrant-local'], 
                   capture_output=True, text=True)
    subprocess.run(['docker', 'rm', 'qdrant-local'], 
                   capture_output=True, text=True)
    
    # Run qdrant container
    cmd = [
        'docker', 'run', '-d', 
        '--name', 'qdrant-local',
        '-p', '6333:6333',
        '-p', '6334:6334',
        'qdrant/qdrant'
    ]
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode == 0:
            print("✅ Local Qdrant container started successfully")
            return True
        else:
            print(f"❌ Failed to start Qdrant container: {result.stderr}")
            return False
    except Exception as e:
        print(f"❌ Error starting Qdrant container: {e}")
        return False

def test_qdrant_connection():
    """Test connection to Qdrant."""
    max_retries = 10
    for i in range(max_retries):
        try:
            response = requests.get('http://localhost:6333/collections')
            if response.status_code == 200:
                print("✅ Successfully connected to local Qdrant")
                return True
        except requests.exceptions.ConnectionError:
            pass
        
        print(f"Waiting for Qdrant to be ready... ({i+1}/{max_retries})")
        time.sleep(2)
    
    print("❌ Failed to connect to Qdrant after multiple attempts")
    return False

def update_env_file():
    """Update the .env file to use local Qdrant."""
    env_path = '../.env'
    
    # Read current .env content
    with open(env_path, 'r') as f:
        lines = f.readlines()
    
    # Update Qdrant settings to use local instance
    updated_lines = []
    for line in lines:
        if line.startswith('QDRANT_URL='):
            updated_lines.append('QDRANT_URL="http://localhost:6333"\n')
        elif line.startswith('QDRANT_API_KEY='):
            updated_lines.append('QDRANT_API_KEY=""\n')  # Local Qdrant doesn't need API key
        else:
            updated_lines.append(line)
    
    # Write updated content back
    with open(env_path, 'w') as f:
        f.writelines(updated_lines)
    
    print("✅ Updated .env file to use local Qdrant")

def main():
    print("Setting up local Qdrant for development...")
    
    if not check_docker():
        print("\nTo use Qdrant, you need to install Docker:")
        print("- Windows: Download Docker Desktop from https://www.docker.com/products/docker-desktop")
        print("- After installation, restart your system and run this script again")
        print("\nAlternatively, you can get a cloud Qdrant instance at https://qdrant.tech/")
        return False
    
    if not start_local_qdrant():
        print("Failed to start local Qdrant instance")
        return False
    
    if not test_qdrant_connection():
        print("Failed to connect to local Qdrant instance")
        return False
    
    update_env_file()
    
    print("\n🎉 Local Qdrant setup completed!")
    print("The RAG system will now work with your local Qdrant instance")
    print("Remember to restart your backend server to pick up the new settings")
    
    return True

if __name__ == "__main__":
    main()