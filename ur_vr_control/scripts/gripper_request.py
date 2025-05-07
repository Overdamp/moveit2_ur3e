import requests
import asyncio
from concurrent.futures import ThreadPoolExecutor


class GripperRequest:

    def __init__(self, host):
        self.host = host
        self.gripper_api_url = "http://" + host + ":8005"
        self.task = None
        self.lock = asyncio.Lock()  # To ensure thread-safe access to tasks

    async def control_gripper(self, action):
        """Control the gripper through its API"""
        try:
            with ThreadPoolExecutor() as executor:
                response = await asyncio.get_event_loop().run_in_executor(
                    executor,
                    lambda: requests.post(f"{self.gripper_api_url}/{action}"),
                )
            return response.json()
        except Exception as e:
            print(f"Error controlling gripper: {e}")
            return None

    async def open_gripper_async(self):
        res = await self.control_gripper("open")
        return res

    async def close_gripper_async(self):
        res = await self.control_gripper("close")
        return res

    def open_gripper(self):
        """Wrapper for opening the gripper."""
        return asyncio.run(self.open_gripper_async())

    def close_gripper(self):
        """Wrapper for closing the gripper."""
        return asyncio.run(self.close_gripper_async())


if __name__ == "__main__":
    import time

    gripper = GripperRequest("130.215.216.42")
    gripper.open_gripper()
    time.sleep(1.5)
    gripper.close_gripper()
    time.sleep(1.5)
    gripper.open_gripper()
