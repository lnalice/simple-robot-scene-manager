import os
from dotenv import load_dotenv

load_dotenv()

DB_HOST = os.getenv('DB_HOST')
DB_USER = os.getenv('DB_USER')
DB_PASSWORD = os.getenv('DB_PASSWORD')
DATABASE = os.getenv('DATABASE')
DB_PORT = os.getenv('DB_PORT')

mysql_config = {
    "host": DB_HOST,
    "user": DB_USER,
    "password": DB_PASSWORD,
    "database": DATABASE
}

bitlog_config = {
    "host": mysql_config["host"],
    "user": mysql_config["user"],
    "passwd": mysql_config["password"],
    "port": DB_PORT
}