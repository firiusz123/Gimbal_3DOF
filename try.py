from itertools import product
options = {
    "Motors": [
        ("Servomotors", 7, 3),
        ("Stepper motor", 5, 4),
        ("DC motor + encoder", 8, 5),
        ("BLDC motor + encoder", 10, 6)
    ],
    "Feedback": [
        ("Accelerometer", 3, 1),
        ("Accelerometer x2", 4, 2),
        ("Magnetometer", 2, 1),
        ("Magnetometer + Accelerometer", 7, 2),
        ("IMU", 8, 3),
        ("IMU x2", 9, 10),
        ("IMU + Camera data", 10, 4)
    ],
    "Control": [
        ("Dedicated PCB", 10, 5),
        ("Market Board", 5, 2)
    ],
    "Image Acquisition": [
        ("Camera", 7, 3),
        ("Camera + Infrared", 10, 6)
    ],
    "Image Transfer": [
        ("Wired", 10, 1),
        ("Wireless", 3, 4)
    ],
    "Transfer Protocol": [
        ("UART", 8, 1),
        ("SPI", 8, 1),
        ("I2C", 6, 2),
        ("ETHERNET", 10, 6)
    ],
    "Water Resistance": [
        ("IP_1", 1, 1),
        ("IP_2", 2, 2),
        ("IP_3", 3, 2),
        ("IP_4", 4, 3),
        ("IP_5", 4, 3),
        ("IP_6", 5, 4),
        ("IP_7", 9, 5),
        ("IP_8", 10, 9)
    ],
    "Dust Resistance": [
        ("IP4", 6, 1),
        ("IP5", 8, 4),
        ("IP6", 10, 8)
    ]
}

best_score = float('-inf')
best_configuration = None

for combination in product(*options.values()):
    functionality = sum(item[1] for item in combination)
    price = sum(item[2] for item in combination)
    score = functionality - price

    if score > best_score:
        best_score = score
        best_configuration = combination

print("Optimal Configuration (Maximizing Functionality while Minimizing Cost):")
for param, option in zip(options.keys(), best_configuration):
    print(f"{param}: {option[0]} (Functionality: {option[1]}, Price: {option[2]})")

print(f"\nTotal Functionality: {sum(item[1] for item in best_configuration)}")
print(f"Total Price: {sum(item[2] for item in best_configuration)}")
print(f"Optimal Score: {best_score}")
