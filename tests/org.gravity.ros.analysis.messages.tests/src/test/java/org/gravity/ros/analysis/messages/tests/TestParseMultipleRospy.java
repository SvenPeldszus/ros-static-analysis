package org.gravity.ros.analysis.messages.tests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import java.util.List;
import java.util.Map;
import org.apache.log4j.Logger;
import org.eclipse.core.runtime.CoreException;
import org.gravity.ros.analysis.messages.Dataclasses.AstModuleInfo;
import org.gravity.ros.analysis.messages.Dataclasses.TopicInfo;
import org.gravity.ros.analysis.messages.PythonProjectParser;
import org.gravity.ros.analysis.messages.UMLZeichner;
import org.junit.Test;
import org.python.pydev.parser.jython.ast.FunctionDef;

public class TestParseMultipleRospy extends TestAbstarctProjectParse {

	/**
	 * The logger of this class
	 */
	private static final Logger LOGGER = Logger.getLogger(TestParseMinimalRospy.class);
	private static String projectName = "PyDevMultipleNodesRosRospy";
	
	public TestParseMultipleRospy() throws CoreException {
		super(projectName);
	}

	@Test
	public void testParseFunctionShouldReturnTwoAsts(){
		System.out.println("Test project: " + project.getName());
		PythonProjectParser parser = new PythonProjectParser();
		List<AstModuleInfo> parsedList;
		
		try {
			parsedList = parser.parse(project);
			
			assertEquals(5, parsedList.size());
		} catch (CoreException e) { LOGGER.error(e); }
	}
	
	@Test 
	public void testGetRosApiMustFindThreeFunctions(){
		PythonProjectParser parser = new PythonProjectParser();
		List<FunctionDef> rosAPI;
			
		try {
			rosAPI = parser.getRosAPI(project);
			
			assertEquals(3, rosAPI.size());
		} catch (Exception e) { e.printStackTrace(); }
	}
	
	@Test
	public void testDataStructureOfProjectShouldBeCorrect() {
		PythonProjectParser parser = new PythonProjectParser();
		List<AstModuleInfo> parsedList;
		List<FunctionDef> rosAPI;
		Map<String, TopicInfo> topicInfo;
		
		try {
			parsedList = parser.parse(project);
			rosAPI = parser.getRosAPI(project);
			topicInfo = parser.getCalls(parsedList, rosAPI);
			
			
			assertEquals(2, topicInfo.size());
			assertNotNull(topicInfo.get("chatter"));
			
			assertEquals(1, topicInfo.get("chatter").publishers.size());
			assertEquals(1, topicInfo.get("chatter").subscribers.size());
			
			assertEquals(1, topicInfo.get("info").publishers.size());
			assertEquals(2, topicInfo.get("info").subscribers.size());
			
			
		} catch (Exception e) { e.printStackTrace(); }
	}
	
	
	@Test
	public void testUML() {
		PythonProjectParser parser = new PythonProjectParser();
		List<AstModuleInfo> parsedList;
		List<FunctionDef> rosAPI;
		Map<String, TopicInfo> topicInfo;
		
		try {
			parsedList = parser.parse(project);
			rosAPI = parser.getRosAPI(project);
			topicInfo = parser.getCalls(parsedList, rosAPI);
			
			UMLZeichner.createDiagram(projectName, topicInfo);
			
		} catch (Exception e) { e.printStackTrace(); }
	}
}