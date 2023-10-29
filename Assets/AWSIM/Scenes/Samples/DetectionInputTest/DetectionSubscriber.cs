using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;
using AWSIM;
using AWSIM.TrafficSimulation;
using System;

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

        [SerializeField] GameObject[] npcVehiclePrefabs;
        private NPCVehicleSpawner npcVehicleSpawner;
        private Dictionary<String, NPCVehicle> npcVehicles;
        private Dictionary<String, GameObject> npcVehiclesModels;

        private Dictionary<String, autoware_auto_perception_msgs.msg.TrackedObject> detectedVehicles;
        private List<String> spawnedVehicles;
        private List<String> currentFrameVehicles;

        [SerializeField] private NPCVehicleConfig vehicleConfig = NPCVehicleConfig.Default();

        [SerializeField] private GameObject spawnObject;

        // Start is called before the first frame update
        void Start()
        {
            var qos = qosSettings.GetQoSProfile();
            npcVehicles = new Dictionary<String, NPCVehicle>();
            npcVehiclesModels = new Dictionary<String, GameObject>();
            detectedVehicles = new Dictionary<String, autoware_auto_perception_msgs.msg.TrackedObject>();
            spawnedVehicles = new List<String>();
            npcVehicleSpawner = new NPCVehicleSpawner(this.gameObject, npcVehiclePrefabs);
            trackedObjectsPublisher = SimulatorROS2Node.CreatePublisher<autoware_auto_perception_msgs.msg.TrackedObjects>(detectedObjectsPublisherTopic, qos);
            trackedObjectsSubscriber
                    = SimulatorROS2Node.CreateSubscription<autoware_auto_perception_msgs.msg.TrackedObjects>(
                        detectedObjectsTopic, msg =>
                        {
                            trackedObjectsPublisher.Publish(msg);
                            currentFrameVehicles = new List<String>();
                            foreach (autoware_auto_perception_msgs.msg.TrackedObject obj in msg.Objects){
                                if (obj.Classification[0].Label == 1 /*&& obj.Existence_probability >= 0.3*/){
                                    string uuid = BitConverter.ToString(obj.Object_id.Uuid).Replace("-", "");
                                    currentFrameVehicles.Add(uuid);
                                    if (detectedVehicles.ContainsKey(uuid)){
                                        detectedVehicles[uuid] = obj;
                                        continue;
                                    }
                                    Debug.Log("New Object ID: " + uuid);
                                    detectedVehicles.Add(uuid, obj);
                                }
                            }

                        }, qos);
        }

        void Update(){
            foreach (String vehicleID in detectedVehicles.Keys){
                if (spawnedVehicles.Contains(vehicleID)){
                    continue;
                }
                spawnedVehicles.Add(vehicleID);
                StartCoroutine(Routine(vehicleID));
            }
        }

        IEnumerator Routine(string vehicleID)
        {
            Vector3 spawnPosition = new Vector3((float)-detectedVehicles[vehicleID].Kinematics.Pose_with_covariance.Pose.Position.Y, 0f, (float)detectedVehicles[vehicleID].Kinematics.Pose_with_covariance.Pose.Position.X);
            Quaternion rotation = new Quaternion(0f, (float)-detectedVehicles[vehicleID].Kinematics.Pose_with_covariance.Pose.Orientation.Z, 0f, (float)detectedVehicles[vehicleID].Kinematics.Pose_with_covariance.Pose.Orientation.W);
            GameObject randomPrefab;
            if (!npcVehiclesModels.ContainsKey(vehicleID))
            {        
                randomPrefab = npcVehicleSpawner.GetRandomPrefab();
                npcVehiclesModels.Add(vehicleID, randomPrefab);
            }
            else 
            {
                randomPrefab = npcVehiclesModels[vehicleID];
            }
            NPCVehicle vehicle = npcVehicleSpawner.Spawn(randomPrefab, SpawnIdGenerator.Generate(), spawnPosition, rotation, spawnObject.transform);
            npcVehicles.Add(vehicleID, vehicle);
            while (true)
            {
                double vehicleSpeed = detectedVehicles[vehicleID].Kinematics.Twist_with_covariance.Twist.Linear.X;
                double vehicleYawSpeed = -detectedVehicles[vehicleID].Kinematics.Twist_with_covariance.Twist.Angular.Z * Mathf.Rad2Deg;
                    
                if (!currentFrameVehicles.Contains(vehicleID))
                {
                    Debug.Log("Despawn vehicle: " + vehicleID);
                    npcVehicleSpawner.Despawn(vehicle);
                    npcVehicles.Remove(vehicleID);
                    spawnedVehicles.Remove(vehicleID);
                    detectedVehicles.Remove(vehicleID);
                    break;
                }

                Vector3 updatedPosition = vehicle.transform.position;
                Quaternion updatedRotation = vehicle.transform.rotation;
                if (vehicleSpeed >= 0)
                {
                    updatedPosition = new Vector3((float)-detectedVehicles[vehicleID].Kinematics.Pose_with_covariance.Pose.Position.Y, 0f, (float)detectedVehicles[vehicleID].Kinematics.Pose_with_covariance.Pose.Position.X);
                    updatedRotation = new Quaternion(0f, (float)-detectedVehicles[vehicleID].Kinematics.Pose_with_covariance.Pose.Orientation.Z, 0f, (float)detectedVehicles[vehicleID].Kinematics.Pose_with_covariance.Pose.Orientation.W);
                }
                yield return UpdatePosAndRot(vehicle, /*0.1f,*/ (float)vehicleSpeed, (float)vehicleYawSpeed, updatedPosition, updatedRotation);
            }
        }

        IEnumerator UpdatePosAndRot(NPCVehicle npcVehicle, /*float duration,*/ float speed, float yawSpeed, Vector3 position, Quaternion rotation)
        {
            var startTime = Time.fixedTime;
            //yield return new WaitForFixedUpdate();
            Vector3 currentPosition = npcVehicle.transform.position;
            Quaternion currentRotation = npcVehicle.transform.rotation;
            //while (Time.fixedTime - startTime < duration)
            //{
                var euler = currentRotation.eulerAngles;
                currentRotation = Quaternion.Euler(euler.x, euler.y + yawSpeed * Time.fixedDeltaTime, euler.z);
                currentPosition += currentRotation * Vector3.forward * speed * Time.fixedDeltaTime;
                if (yawSpeed > 10f){
                    npcVehicle.SetTurnSignalState(NPCVehicle.TurnSignalState.RIGHT);
                }
                else if (yawSpeed < -10f){
                    npcVehicle.SetTurnSignalState(NPCVehicle.TurnSignalState.LEFT);
                }
                else{
                    npcVehicle.SetTurnSignalState(NPCVehicle.TurnSignalState.OFF);
                }
                //npcVehicle.gameObject.transform.forward = rotation * Vector3.forward;
                if (speed > 1f){
                    npcVehicle.SetRotation(rotation);
                    npcVehicle.SetPosition(position); 
                }
                else{
                    npcVehicle.SetRotation(currentRotation);
                    npcVehicle.SetPosition(currentPosition); 
                }

                yield return new WaitForFixedUpdate();
            //}
        }

        void OnDestroy(){
            SimulatorROS2Node.RemoveSubscription<autoware_auto_perception_msgs.msg.TrackedObjects>(trackedObjectsSubscriber);
            npcVehicles.Clear();
            detectedVehicles.Clear();
            spawnedVehicles.Clear();
        }
    }
}