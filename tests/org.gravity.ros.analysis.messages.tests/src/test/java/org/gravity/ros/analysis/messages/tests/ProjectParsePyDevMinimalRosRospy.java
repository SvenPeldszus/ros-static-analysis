package org.gravity.ros.analysis.messages.tests;

import static org.junit.Assert.assertEquals;

import java.net.URISyntaxException;
import java.util.List;
import org.apache.log4j.Logger;
import org.eclipse.core.runtime.CoreException;
import org.gravity.ros.analysis.messages.PythonProjectParser;
import org.junit.Test;
import org.python.pydev.parser.jython.ast.FunctionDef;

public class ProjectParsePyDevMinimalRosRospy extends AbstarctProjectParseTest {

	/**
	 * The logger of this class
	 */
	private static final Logger LOGGER = Logger.getLogger(ProjectParseTestPyDevPythonProject.class);
	
	public ProjectParsePyDevMinimalRosRospy() throws CoreException {
		super("PyDevMinimalRosRospy");
	}

	@Test
	public void testDummy(){
		System.out.println("Test project: " + project.getName());
		try {
			PythonProjectParser parser = new PythonProjectParser();
			var parsedList = parser.parse(project);

			List<FunctionDef> ros = parser.getRosAPI(parsedList);

			assertEquals(3, ros.size());

		} catch (CoreException e) {
			LOGGER.error(e);
		}
	}

//	@Test
//	public void testDummy2() {
//		System.out.println("Test project: " + project.getName());
//		try {
//			PythonProjectParser parser = new PythonProjectParser();
//			var parsedList = parser.parse(project);
//			
//			/* Brauchen wir nicht */
//			//parseContentPrint(parsedList);
//
//			List<FunctionDef> ros = parser.getRosAPI(parsedList);
//
//			assertEquals(2, ros.size());
//			parser.getCalls(parsedList, ros);
//
//		} catch (CoreException e) {
//			LOGGER.error(e);
//		}
//	}
}