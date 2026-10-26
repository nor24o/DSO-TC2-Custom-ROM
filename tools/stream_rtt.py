import sys
import time
import pylink

def stream_rtt(duration=10):
    jlink = pylink.JLink()
    try:
        jlink.open(serial_no=69650043)
        jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
        jlink.connect('STM32L083RZ', speed=4000)
        jlink.rtt_start()
        print("[+] J-Link RTT connected successfully! Streaming live data...\n")
        
        start = time.time()
        while time.time() - start < duration:
            data = jlink.rtt_read(0, 1024)
            if data:
                text = bytes(data).decode('utf-8', errors='ignore')
                sys.stdout.write(text)
                sys.stdout.flush()
            time.sleep(0.01)
    except Exception as e:
        print(f"[-] Error: {e}")
    finally:
        try:
            jlink.rtt_stop()
            jlink.close()
        except:
            pass

if __name__ == "__main__":
    dur = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    stream_rtt(dur)
