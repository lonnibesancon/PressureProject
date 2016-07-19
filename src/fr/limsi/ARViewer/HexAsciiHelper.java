package fr.limsi.ARViewer;

import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.TextView;
import java.nio.charset.StandardCharsets;
import android.util.Log;

import java.util.regex.Pattern;

public class HexAsciiHelper {
    public static int PRINTABLE_ASCII_MIN = 0x20; // ' '
    public static int PRINTABLE_ASCII_MAX = 0x7E; // '~'

    public static boolean isPrintableAscii(int c) {
        return c >= PRINTABLE_ASCII_MIN && c <= PRINTABLE_ASCII_MAX;
    }

    public static String bytesToHex(byte[] data) {
        return bytesToHex(data, 0, data.length);
    }

    public static String bytesToHex(byte[] data, int offset, int length) {
        if (length <= 0) {
            return "";
        }

        StringBuilder hex = new StringBuilder();
        for (int i = offset; i < offset + length; i++) {
            hex.append(String.format(" %02X", data[i] % 0xFF));
        }
        hex.deleteCharAt(0);
        return hex.toString();
    }

    public static String bytesToAsciiMaybe(byte[] data) {
        return bytesToAsciiMaybe(data, 0, data.length);
    }

    private static String[] copyReverse(String[] tab){
        for(int i = 0; i < tab.length / 2; i++)
        {
            String temp = tab[i];
            tab[i] = tab[tab.length - i - 1];
            tab[tab.length - i - 1] = temp;
        }
        return tab ;
    }

    public static int HexToInt(String hex){
        String[] spl = hex.split(" ");
        for(int i = 0 ; i < spl.length ; i++){
            if(spl[i].length() > 2){        //Hardcoded because of weird values
                int len = spl[i].length() ;
                String tmp = new StringBuilder().append(spl[i].charAt(len-2)).append(spl[i].charAt(len-1)).toString();
                spl[i] = tmp ;
            }
        }
        spl = copyReverse(spl);
        String result = "";
        for(int i = 0 ; i < spl.length ; i++){
            result += spl[i];
        }
        Long number = Long.parseLong(result,16);
        int decimal = number != null ? number.intValue() : null;


        return decimal;
    }

    public static float HexToFloat(String hex){
        String[] spl = hex.split(" ");
        for(int i = 0 ; i < spl.length ; i++){
            if(spl[i].length() > 2){        //Hardcoded because of weird values
                int len = spl[i].length() ;
                String tmp = new StringBuilder().append(spl[i].charAt(len-2)).append(spl[i].charAt(len-1)).toString();
                spl[i] = tmp ;
            }
        }
        spl = copyReverse(spl);
        String result = "";
        for(int i = 0 ; i < spl.length ; i++){
            result += spl[i];
        }
        Long i = Long.parseLong(result, 16);
        float decimal = Float.intBitsToFloat(i.intValue());
        Log.d("FloatValue",""+decimal);


        return decimal ;
    }

    public static String bytesToAsciiMaybe(byte[] data, int offset, int length) {
        StringBuilder ascii = new StringBuilder();
        boolean zeros = false;
        for (int i = offset; i < offset + length; i++) {
            int c = data[i] & 0xFF;
            if (isPrintableAscii(c)) {
                if (zeros) {
                    return null;
                }
                ascii.append((char) c);
            } else if (c == 0) {
                zeros = true;
            } else {
                return null;
            }
        }
        return ascii.toString();
    }

    public static byte[] hexToBytes(String hex) {
        ByteArrayBuffer bytes = new ByteArrayBuffer(hex.length() / 2);
        //String str = new String(bytes, StandardCharsets.UTF_8);
        //Log.d("BYTES",""+bytes);
        for (int i = 0; i < hex.length(); i++) {
            if (hex.charAt(i) == ' ') {
                continue;
            }

            String hexByte;
            if (i + 1 < hex.length()) {
                hexByte = hex.substring(i, i + 2).trim();
                i++;
            } else {
                hexByte = hex.substring(i, i + 1);
            }

            bytes.append(Integer.parseInt(hexByte, 16));
        }
        return bytes.buffer();
    }
}
