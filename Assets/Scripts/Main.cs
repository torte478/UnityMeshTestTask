using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Main : MonoBehaviour
{
    public Solver Solver;

    void Start()
    {
        var gui = GetComponent<GUI>();
        gui.InitInputFields(20, 30, 40);
        gui.OnInputChange += OnInputChange;

        OnInputChange(20, 30, 40);
    }

    private void OnInputChange(int offset, int angle, int shift)
    {
        

        GetComponent<GUI>().UpdateResultText(angle.ToString());
    }
}
