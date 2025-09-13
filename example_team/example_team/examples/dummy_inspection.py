import asyncio

import rclpy
from rclpy.time import Duration
from rclpy.executors import MultiThreadedExecutor

from ariac_interfaces.msg import CellTypes, InspectionReport

from example_team.environment_interface import Environment
from example_team.sensors_interface import Sensors
from example_team.utils import AsyncUtils

async def run():
    rclpy.init()

    # Create and spin executor
    executor = MultiThreadedExecutor()
    shutdown_event = asyncio.Event()
    spin_task = asyncio.create_task(AsyncUtils.spin_executor(executor, shutdown_event))

    environment = Environment()
    sensors = Sensors()
    executor.add_node(environment)
    executor.add_node(sensors)

    try:
        environment.get_logger().info("Waiting for enviornment to be ready")
        await environment.competition.ready.wait()

        environment.get_logger().info("Starting competition")
        await environment.competition.start()

        environment.get_logger().info("Starting cell feed")
        await environment.inspection_conveyor.start_cell_feed(CellTypes.LI_ION)

        for _ in range(5):
            cell = await sensors.inspection_bb.cell_queue.get()

            environment.get_logger().info(f"Cell detected at {cell.detection_time.nanoseconds/1E9} seconds")

            # Simulate inspection time
            await AsyncUtils.await_for_duration(environment.get_clock(), Duration(seconds=1))

            environment.get_logger().info("Submitting inspection report")
            report = InspectionReport(passed=True)
            await environment.inspection_conveyor.submit_inspection_report(report)

        await environment.inspection_conveyor.stop_cell_feed()

        await environment.competition.end(shutdown=False)      

    except Exception as e:
        print(5*'\n', e, 5*'\n')
    finally:
        shutdown_event.set()
        await spin_task  

def main():
    asyncio.run(run())