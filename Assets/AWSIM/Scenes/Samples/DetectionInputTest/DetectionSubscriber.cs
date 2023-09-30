using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;
using AWSIM;

namespace AWSIM.Tests
{
    public class DetectionSubscriber : MonoBehaviour
    {

        [SerializeField] string detectedObjectsTopic = "/perception/object_recognition/tracking/objects";
        [SerializeField] string detectedObjectsPublisherTopic = "/unity/perception/object_recognition/tracking/objects";
        [SerializeField] QoSSettings qosSettings = new QoSSettings(){
                ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
                DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
                HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
                Depth = 5,
        };

        ISubscription<autoware_auto_perception_msgs.msg.TrackedObjects> trackedObjectsSubscriber;
        IPublisher<autoware_auto_perception_msgs.msg.TrackedObjects> trackedObjectsPublisher;

        // Start is called before the first frame update
        void Start()
        {
            var qos = qosSettings.GetQoSProfile();
            trackedObjectsPublisher = SimulatorROS2Node.CreatePublisher<autoware_auto_perception_msgs.msg.TrackedObjects>(detectedObjectsPublisherTopic, qos);

            trackedObjectsSubscriber
                    = SimulatorROS2Node.CreateSubscription<autoware_auto_perception_msgs.msg.TrackedObjects>(
                        detectedObjectsTopic, msg =>
                        {
                            trackedObjectsPublisher.Publish(msg);
                        }, qos);
        }

        // bool NeedToPublish()
        // {
        //         timer += Time.deltaTime;
        //         var interval = 1.0f / publishHz;
        //         interval -= 0.00001f;
        //         if (timer < interval)
        //             return false;
        //         timer = 0;
        //         return true;
        // }

        // // Update is called once per frame
        // void FixedUpdate()
        // {
        //     if (Nee)
        // }

        void OnDestroy(){
            SimulatorROS2Node.RemoveSubscription<autoware_auto_perception_msgs.msg.TrackedObjects>(trackedObjectsSubscriber);
        }
    }
}