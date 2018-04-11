using UnityEngine;
using System.Collections;
using UnityEngine.UI;

public class ToggleButton : MonoBehaviour
{

    public bool ButtonOn = false;
    public Button MyButton;

    public void BeenClicked()
    {
        ButtonOn = !ButtonOn;
        if (ButtonOn)
        {
            MyButton.image.color = Color.red;
        }
        else
        {
            MyButton.image.color = Color.white;
        }
    }
}