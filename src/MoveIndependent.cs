using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoveIndependent : MonoBehaviour
{
    Quaternion startRotation;
    Quaternion endRotation;
    Vector3 startPosition;
    Vector3 endPosition;
    LineRenderer lr0;

    int totalFrames = 50*10;
    int counter;

    // Start is called before the first frame update
    void Start()
    {
        lr0 = new GameObject().AddComponent<LineRenderer>();
        lr0.gameObject.transform.SetParent(transform);
        lr0.gameObject.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);

        startRotation = Quaternion.Euler(0f, 0f, 0f);
        startPosition = Vector3.zero;

        endPosition = new Vector3(-10f, 10f, 10f);
        endRotation = Quaternion.Euler(180f,0f,0f);

        counter = 0;

        transform.position = startPosition;
        transform.rotation = startRotation;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void FixedUpdate() {
        if (counter < totalFrames) {
            counter += 1;
            transform.rotation = Quaternion.Slerp(startRotation, endRotation, (float)counter / totalFrames);
            transform.position = Vector3.Lerp(startPosition, endPosition, (float) counter / totalFrames);

            List<Vector3> line0 = new List<Vector3> { startPosition, endPosition };
            lr0.startWidth = 0.1f;
            lr0.endWidth = 0.1f;
            lr0.SetPositions(line0.ToArray());
        }
        
    }
}
