# --- Variables ---

BAG_NAME = 'longitudal_3'  # Change only this

BAG_PATH = 'rosbags/' + BAG_NAME + '/' + BAG_NAME + '_0.db3'
PATH = 'rosbags/' + BAG_NAME + '/'
IS_SERVO = False  # Will be updated by check_servo_topic_exists()


# --- Functions ---
def check_servo_topic_exists() -> None:
    """Checks if '/commands/servo/position' exists in the bag and updates IS_SERVO."""
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

    global IS_SERVO

    storage_options = StorageOptions(uri=BAG_PATH, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    all_topics = [info.name for info in reader.get_all_topics_and_types()]
    IS_SERVO = '/commands/servo/position' in all_topics

    if IS_SERVO:
        print("[INFO] Servo topic found. IS_SERVO = True")
    else:
        print("[INFO] Servo topic NOT found. IS_SERVO = False")
