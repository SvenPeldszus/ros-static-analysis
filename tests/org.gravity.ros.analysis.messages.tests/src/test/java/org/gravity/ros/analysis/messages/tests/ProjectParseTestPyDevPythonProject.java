package org.gravity.ros.analysis.messages.tests;

import static org.junit.Assert.assertEquals;

import java.util.List;
import org.apache.log4j.Logger;
import org.eclipse.core.runtime.CoreException;
import org.gravity.ros.analysis.messages.PythonProjectParser;
import org.junit.Test;
import org.python.pydev.parser.jython.ast.FunctionDef;

public class ProjectParseTestPyDevPythonProject extends AbstarctProjectParseTest {

	/**
	 * The logger of this class
	 */
	private static final Logger LOGGER = Logger.getLogger(ProjectParseTestPyDevPythonProject.class);
	
	public ProjectParseTestPyDevPythonProject() throws CoreException {
		super("PyDevPythonProject");
	}

	@Test
	public void testDummy() {
		System.out.println("Test project: " + project.getName());
		try {
			PythonProjectParser parser = new PythonProjectParser();
			var parsedList = parser.parse(project);
			/* NEW TEST FOR PARSE OUTPUT */
			parseContentPrint(parsedList);

			List<FunctionDef> ros = parser.getRosAPI(parsedList);

			assertEquals(1, ros.size());

		} catch (CoreException e) {
			LOGGER.error(e);
		}
	}

	@Test
	public void testDummy2() {
		System.out.println("Test project: " + project.getName());
		try {
			PythonProjectParser parser = new PythonProjectParser();
			var parsedList = parser.parse(project);
			/* NEW TEST FOR PARSE OUTPUT */
			parseContentPrint(parsedList);

			List<FunctionDef> ros = parser.getRosAPI(parsedList);

			assertEquals(2, ros.size());

		} catch (CoreException e) {
			LOGGER.error(e);
		}
	}
}