package frc.robot.lib.BLine;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.nio.file.Path;
import javax.xml.parsers.DocumentBuilderFactory;
import org.junit.jupiter.api.Test;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

class PublicationMetadataTest {
    @Test
    void publishedPomExposesCommandsV2TransitivelyToConsumers() throws Exception {
        var document = DocumentBuilderFactory.newInstance()
            .newDocumentBuilder()
            .parse(Path.of("build/publications/maven/pom-default.xml").toFile());
        NodeList dependencies = document.getElementsByTagName("dependency");

        for (int index = 0; index < dependencies.getLength(); index++) {
            Element dependency = (Element) dependencies.item(index);
            if ("org.wpilib.commandsv2".equals(text(dependency, "groupId"))
                && "commandsv2-java".equals(text(dependency, "artifactId"))) {
                assertEquals("compile", text(dependency, "scope"));
                return;
            }
        }

        throw new AssertionError("Published POM does not contain Commands v2");
    }

    private static String text(Element element, String tagName) {
        return element.getElementsByTagName(tagName).item(0).getTextContent();
    }
}
