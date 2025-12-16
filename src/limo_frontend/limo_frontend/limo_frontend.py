import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import streamlit as st
import json
from typing import Optional, Dict, Any


#ROS2 Subscriber for Medical Documentation with Streamlit Web Interface.

#This module subscribes to the 'final_medical_documentation' topic from
#limo_mission_finalizer and displays the data in a Streamlit webpage.




class MedicalDocumentationSubscriber(Node):
    """
    ROS2 Node that subscribes to medical documentation topic.
    
    Attributes:
        latest_data (Optional[Dict[str, Any]]): Stores the most recent message data.
    """
    
    def __init__(self) -> None:
        """Initialize the ROS2 node and create subscription."""
        super().__init__('limo_frontend_node')
        self.latest_data: Optional[Dict[str, Any]] = None
        
        # Subscribe to the medical documentation topic
        self.subscription = self.create_subscription(
            String,
            '/hospital/send_diagnosis_email',
            self.listener_callback,
            10
        )
        self.get_logger().info('Subscribed to /hospital/send_diagnosis_email topic')
    
    def listener_callback(self, msg: String) -> None:
        """
        Callback function for incoming messages.
        
        Args:
            msg (String): The incoming message containing JSON medical data.
        """
        try:
            self.latest_data = json.loads(msg.data)
            self.get_logger().info(f'Received: {self.latest_data}')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON: {e}')


def main() -> None:
    if not rclpy.ok():
        rclpy.init()

    if 'subscriber' not in st.session_state:
        st.session_state.subscriber = MedicalDocumentationSubscriber()

    subscriber = st.session_state.subscriber

    # Streamlit page configuration
    st.set_page_config(page_title="Medical Documentation", layout="centered")
    st.title("🏥 Medical Documentation")
    
    # Placeholder for updates
    placeholder = st.empty()
    

    rclpy.spin_once(subscriber, timeout_sec=0.1)
        
    if subscriber.latest_data:
            with placeholder.container():
                # Display patient information (title section)
                st.subheader(f"👤 {subscriber.latest_data.get('name', 'N/A')}, Age: {subscriber.latest_data.get('age', 'N/A')}")
                
                # Display medical treatment (text section)
                st.write("**Medical Treatment:**")
                st.text_area(
                    label="Treatment Details",
                    value=subscriber.latest_data.get('medical_treatment', 'No data available'),
                    disabled=True,
                    height=300
                )
    
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()