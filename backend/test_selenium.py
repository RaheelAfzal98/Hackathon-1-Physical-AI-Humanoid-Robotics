from selenium import webdriver
from selenium.webdriver.chrome.options import Options

# Test if selenium works
chrome_options = Options()
chrome_options.add_argument("--headless")
chrome_options.add_argument("--no-sandbox")
chrome_options.add_argument("--disable-dev-shm-usage")
chrome_options.add_argument("--disable-gpu")
chrome_options.add_argument("--window-size=1920,1080")

try:
    driver = webdriver.Chrome(options=chrome_options)
    driver.get("https://www.google.com")
    print("Selenium is working! Page title:", driver.title)
    driver.quit()
except Exception as e:
    print("Selenium error:", str(e))
    print("You may need to install ChromeDriver")