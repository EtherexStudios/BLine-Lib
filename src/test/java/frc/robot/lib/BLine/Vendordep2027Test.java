package frc.robot.lib.BLine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import java.nio.file.Files;
import java.nio.file.Path;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.junit.jupiter.api.Test;

class Vendordep2027Test {
    private static final String BLINE_UUID = "4b7270e9-4e8d-4e7b-8cf0-5805f12c3c7d";
    private static final String SOURCE_COMMIT = "4dd378b77a0ec73d4c89efb752756728d138801a";

    @Test
    void compatibilityVendordepIdentifiesThe2027LineAndExactSourceCommit() throws Exception {
        JSONObject stable = parse("BLine-Lib.json");
        JSONObject compatibility = parse("BLine-Lib-2027.json");

        assertEquals("BLine-Lib-2027.json", compatibility.get("fileName"));
        assertEquals("0.9.1-wpilib2027.alpha06.01", compatibility.get("version"));
        assertEquals("2027_alpha5", compatibility.get("wpilibYear"));
        assertEquals(BLINE_UUID, compatibility.get("uuid"));
        assertNotEquals(stable.get("jsonUrl"), compatibility.get("jsonUrl"));
        assertEquals(
            "https://raw.githubusercontent.com/edanliahovetsky/BLine-Lib/wpilib-2027/BLine-Lib-2027.json",
            compatibility.get("jsonUrl")
        );

        JSONArray dependencies = (JSONArray) compatibility.get("javaDependencies");
        assertEquals(1, dependencies.size());
        JSONObject bline = (JSONObject) dependencies.getFirst();
        assertEquals("com.github.edanliahovetsky", bline.get("groupId"));
        assertEquals("BLine-Lib", bline.get("artifactId"));
        assertEquals(SOURCE_COMMIT, bline.get("version"));
    }

    private static JSONObject parse(String fileName) throws Exception {
        return (JSONObject) new JSONParser().parse(Files.readString(Path.of(fileName)));
    }
}
