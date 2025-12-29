import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__)))

try:
    print("Attempting to import the main app...")
    from src.main import app
    print("App imported successfully!")
    
    print("Checking if server can start...")
    import uvicorn
    print("Uvicorn is available")
    
    print("All imports successful. The server should start properly.")
    
except Exception as e:
    print(f"Error importing or checking app: {e}")
    import traceback
    traceback.print_exc()