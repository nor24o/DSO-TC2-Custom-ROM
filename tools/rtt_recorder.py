import sys
import time
import pylink

def main():
    log_file = "rtt_button_log.txt"
    with open(log_file, "w") as f:
        f.write("=== RTT LIVE KEYPAD RECORDER STARTED ===\n")
        
    jlink = pylink.JLink()
    try:
        jlink.open(serial_no=69650043)
        jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
        jlink.connect('STM32L083RZ', speed=4000)
        jlink.rtt_start()
        print(f"[+] Recording RTT to {log_file}...")
        
        while True:
            data = jlink.rtt_read(0, 1024)
            if data:
                text = bytes(data).decode('utf-8', errors='ignore')
                with open(log_file, "a") as f:
                    f.write(text)
                sys.stdout.write(text)
                sys.stdout.flush()
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n[+] Stopped.")
    except Exception as e:
        print(f"[-] Error: {e}")
    finally:
        try:
            jlink.rtt_stop()
            jlink.close()
        except:
            pass

if __name__ == "__main__":
    main()
