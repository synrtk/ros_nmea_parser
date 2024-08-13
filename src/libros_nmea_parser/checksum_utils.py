
def check_nmea_checksum(nmea_sentence):
    """Calculate and compare the checksum of a NMEA string.

    Args:
        nmea_sentence (str): The NMEA sentence to check.

    Return True if the calculated checksum of the sentence matches the one provided.
    """
    split_sentence = nmea_sentence.split('*')
    if len(split_sentence) != 2:
        # No checksum bytes were found... improperly formatted/incomplete NMEA data?
        return False
    transmitted_checksum = split_sentence[1].strip()

    # Remove the $ at the front
    data_to_checksum = split_sentence[0][1:]
    checksum = 0
    for c in data_to_checksum:
        checksum ^= ord(c)

    return ("%02X" % checksum) == transmitted_checksum.upper()
