using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DualQuat
{
    public DualQuat(Quaternion real, Quaternion dual) {
        Real = real;
        Dual = dual;
    }

    public Quaternion Real { get; set; }
    public Quaternion Dual { get; set; }

    public static DualQuat operator * (DualQuat lhs, DualQuat rhs) {
        return new DualQuat(lhs.Real*rhs.Real, QuaternionAdd(lhs.Real*rhs.Dual,rhs.Real*lhs.Dual));
    }

    public static void ToScrew(DualQuat dualQuat, out float theta, out Vector3 axis, out float d, out Vector3 moment) {
        var qr = dualQuat.Real;
        var qd = dualQuat.Dual;
        axis = new Vector3(qr.x, qr.y, qr.z).normalized;
        theta = 2 * Mathf.Acos(qr.w);
        d = -2 * qd.w / Mathf.Sin(theta / 2f);
        moment = (new Vector3(qd.x, qd.y, qd.z) - ((d / 2f) * Mathf.Sin(theta / 2f)) * axis) / Mathf.Sin(theta / 2f);
    }

    public static DualQuat Power(DualQuat dualQuat, float n) {
        ToScrew(dualQuat, out var theta, out var axis, out var d, out var moment);
        var halfAngle = n * theta / 2f;
        var halfD = n * d / 2f;
        var qrScalar = Mathf.Cos(halfAngle);
        var qrVector = axis * Mathf.Sin(halfAngle);
        var qr = new Quaternion(qrVector.x, qrVector.y, qrVector.z, qrScalar);
        var qdScalar = -halfD * Mathf.Sin(halfAngle);
        var qdVector = halfD * Mathf.Cos(halfAngle) * axis + Mathf.Sin(halfAngle) * moment;
        var qd = new Quaternion(qdVector.x, qdVector.y, qdVector.z, qdScalar);
        return new DualQuat(qr, qd);
    }

    public static DualQuat Interpolate(DualQuat start, DualQuat end, float n) {
        return Power(end * Conjugate(start),n)*start;
    }

    public static DualQuat FromPose(Quaternion rotation, Vector3 position) {
        Quaternion t = new Quaternion(position.x, position.y, position.z, 0);
        var dual = t * rotation;
        DualQuat dq = new DualQuat(rotation, ScaleQuaternion(dual, 0.5f));
        return dq;
    }

    public static void ToPose(DualQuat dualQuat, out Quaternion rotation, out Vector3 position) {
        rotation = dualQuat.Real;
        var qTrans = ScaleQuaternion(dualQuat.Dual * QuaternionConjugate(dualQuat.Real), 2f);
        position = new Vector3(qTrans.x, qTrans.y, qTrans.z);
    }

    private static Quaternion ScaleQuaternion(Quaternion q, float scale) {
        return new Quaternion(q.x * scale, q.y * scale, q.z * scale, q.w * scale);
    }

    private static Quaternion QuaternionAdd(Quaternion lhs, Quaternion rhs) {
        return new Quaternion(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z, lhs.w + rhs.w);
    }

    private static Quaternion QuaternionConjugate(Quaternion q) {
        return new Quaternion(-q.x, -q.y, -q.z, q.w);
    }

    private static DualQuat Conjugate(DualQuat dualQuat) {
        return new DualQuat(QuaternionConjugate(dualQuat.Real), QuaternionConjugate(dualQuat.Dual));
    }
}

public class MoveDQ : MonoBehaviour
{
    Quaternion startRotation;
    Quaternion endRotation;
    Vector3 startPosition;
    Vector3 endPosition;
    DualQuat startDQ;
    DualQuat endDQ;
    LineRenderer lr0;
    LineRenderer lr1;
    LineRenderer lr2;

    int totalFrames = 50*10;
    int counter;

    // Start is called before the first frame update
    void Start() {
        lr0 = new GameObject().AddComponent<LineRenderer>();
        lr0.gameObject.transform.SetParent(transform);
        lr0.gameObject.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);

        lr1 = new GameObject().AddComponent<LineRenderer>();
        lr1.gameObject.transform.SetParent(transform);
        lr1.gameObject.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);

        lr2 = new GameObject().AddComponent<LineRenderer>();
        lr2.gameObject.transform.SetParent(transform);
        lr2.gameObject.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);

        startRotation = Quaternion.Euler(0f, 0f, 0f);
        startPosition = Vector3.zero;
        startDQ = DualQuat.FromPose(startRotation, startPosition);

        endPosition = new Vector3(-10f, 10f, 10f);
        endRotation = Quaternion.Euler(180f, 0f, 0f);
        endDQ = DualQuat.FromPose(endRotation, endPosition);

        counter = 0;

        transform.position = startPosition;
        transform.rotation = startRotation;

        Debug.Log($"Start[0.0] => q_real:[{startDQ.Real}]; q_dual:[{startDQ.Dual}]");
        Debug.Log($"End[1.0] => q_real:[{endDQ.Real}]; q_dual:[{endDQ.Dual}]");
    }
    
    // Update is called once per frame
    void Update()
    {
        
    }

    private void FixedUpdate() {
        if (counter < totalFrames) {
            counter += 1;
            var dq = DualQuat.Interpolate(startDQ, endDQ, (float)counter / totalFrames);
            DualQuat.ToPose(dq, out var rotation, out var position);
            transform.rotation = rotation;
            transform.position = position;

            DualQuat.ToScrew(endDQ, out var theta, out var axis, out var d, out var moment);
            var p = Vector3.Cross(axis, moment);
            var pStart = p - 20 * axis;
            var pEnd = p + 10 * axis;

            List<Vector3> line0 = new List<Vector3> { startPosition, endPosition };
            lr0.startWidth = 0.1f;
            lr0.endWidth = 0.1f;
            lr0.SetPositions(line0.ToArray());

            List<Vector3> line1 = new List<Vector3> { pStart, pEnd };
            lr1.startWidth = 0.1f;
            lr1.endWidth = 0.1f;
            lr1.SetPositions(line1.ToArray());

            Vector3 temp = position - pStart;
            var offset = Vector3.Dot(temp, axis);
            var shortestIntersect = pStart + offset * axis;
            List<Vector3> line2 = new List<Vector3> { position, shortestIntersect };
            lr2.startWidth = 0.1f;
            lr2.endWidth = 0.1f;
            lr2.SetPositions(line2.ToArray());

            var distance = Vector3.Distance(position, shortestIntersect);
            Debug.Log($"Distance to screw axis [{distance}]");
        }
        
    }
}
