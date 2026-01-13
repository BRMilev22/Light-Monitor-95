-- Light-Level Monitoring System Database
-- Run: mysql -u root -p < db.sql

CREATE DATABASE IF NOT EXISTS light_monitor;
USE light_monitor;

-- Users table for authentication
CREATE TABLE IF NOT EXISTS users (
    id INT AUTO_INCREMENT PRIMARY KEY,
    username VARCHAR(50) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Light readings log (optional, for history)
CREATE TABLE IF NOT EXISTS light_readings (
    id INT AUTO_INCREMENT PRIMARY KEY,
    light_level INT NOT NULL,
    threshold INT NOT NULL,
    led_state BOOLEAN NOT NULL,
    recorded_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- System settings
CREATE TABLE IF NOT EXISTS settings (
    id INT AUTO_INCREMENT PRIMARY KEY,
    setting_key VARCHAR(50) UNIQUE NOT NULL,
    setting_value VARCHAR(255) NOT NULL
);

-- Default threshold setting
INSERT INTO settings (setting_key, setting_value) 
VALUES ('threshold', '768')
ON DUPLICATE KEY UPDATE setting_value = setting_value;
