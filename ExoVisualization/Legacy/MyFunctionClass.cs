using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class MyFunctionClass : MonoBehaviour {

    public static IEnumerator FadeSpriteColor(SpriteRenderer sr, Color toColor, float fadeDuration)
    {
        Color fromColor = sr.color;
        float elapsedTime = 0;
        while (elapsedTime < fadeDuration)
        {
            float t = elapsedTime / fadeDuration;
            Color tweenColor = Color.Lerp(fromColor, toColor, t);
            sr.color = tweenColor;
            elapsedTime += Time.deltaTime;
            yield return null;
        }
        sr.color = toColor;
    }

    public static IEnumerator FadeTextColor(Text text, Color toColor, float fadeDuration)
    {
        Color fromColor = text.color;
        float elapsedTime = 0;
        while (elapsedTime < fadeDuration)
        {
            float t = elapsedTime / fadeDuration;
            Color tweenColor = Color.Lerp(fromColor, toColor, t);
            text.color = tweenColor;
            elapsedTime += Time.deltaTime;
            yield return null;
        }
        text.color = toColor;
    }

}
