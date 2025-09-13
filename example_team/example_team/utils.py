import asyncio
from rclpy.clock import Clock
from rclpy.time import Time, Duration
from rclpy.client import Client
from rclpy.task import Future as RosFuture
from rclpy.action import ActionClient
from rclpy.executors import Executor

class AsyncUtils:
    @staticmethod
    async def await_action_ready(action_client: ActionClient, timeout: float = 5.0, poll_interval: float = 0.1):
        """Wait until the action server is ready, or timeout."""
        start_time = asyncio.get_running_loop().time()
        while not action_client.server_is_ready():
            if (asyncio.get_running_loop().time() - start_time) > timeout:
                raise TimeoutError(f"Timed out waiting for action server {action_client._action_name} to become available")
            await asyncio.sleep(poll_interval)

    @staticmethod
    async def await_service_ready(client: Client, timeout: float = 5.0, poll_interval: float = 0.1):
        """Wait until the ROS service is ready, or timeout."""
        start_time = asyncio.get_running_loop().time()
        while not client.service_is_ready():
            if (asyncio.get_running_loop().time() - start_time) > timeout:
                raise TimeoutError(f"Timed out waiting for service {client.srv_name} to become available")
            await asyncio.sleep(poll_interval)

    @staticmethod
    async def await_service_response(client: Client, request, timeout: float = 5.0):
        """Send a ROS service request and await the response asynchronously."""
        future = AsyncUtils._ros_to_asyncio_future(client.call_async(request))

        try:
            return await asyncio.wait_for(future, timeout=timeout)
        except asyncio.TimeoutError:
            raise TimeoutError(f"Timed out waiting for response from {client.srv_name} service.")

    @staticmethod
    async def await_until_time(clock: Clock, target_time: Time, poll_interval: float = 0.1):
        """Wait until the ROS clock reaches a target time."""
        while clock.now() < target_time:
            remaining_ns = (target_time - clock.now()).nanoseconds
            remaining_s = max(remaining_ns / 1e9, 0.0)
            await asyncio.sleep(min(remaining_s, poll_interval))

    @staticmethod
    async def await_for_duration(clock: Clock, duration: Duration, poll_interval: float = 0.1):
        """Wait for the given ROS Duration to elapse, accounting for simulated time."""
        start_time = clock.now()
        target_time = start_time + duration
        while clock.now() < target_time:
            remaining_ns = (target_time - clock.now()).nanoseconds
            remaining_s = max(remaining_ns / 1e9, 0.0)
            await asyncio.sleep(min(remaining_s, poll_interval))

    @staticmethod
    def _ros_to_asyncio_future(ros_future: RosFuture) -> asyncio.Future:
        """Convert a ROS 2 rclpy Future into an asyncio Future."""
        loop = asyncio.get_running_loop()
        asyncio_future = loop.create_future()

        def _on_ros_future_complete(fut: RosFuture):
            if fut.cancelled():
                asyncio_future.cancel()
            elif fut.done():
                exc = fut.exception()
                if exc is not None:
                    asyncio_future.set_exception(exc)
                else:
                    asyncio_future.set_result(fut.result())

        ros_future.add_done_callback(_on_ros_future_complete)
        return asyncio_future
    
    @staticmethod
    async def spin_executor(executor: Executor, shutdown_event: asyncio.Event):
        while not shutdown_event.is_set():
            executor.spin_once(timeout_sec=1)
            await asyncio.sleep(0.0)
