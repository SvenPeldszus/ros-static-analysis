package org.gravity.ros.analysis.messages.tests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import org.apache.log4j.Logger;
import org.eclipse.core.runtime.CoreException;
import org.gravity.ros.analysis.messages.Dataclasses.AstModuleInfo;
import org.gravity.ros.analysis.messages.Dataclasses.FunctionInfo;
import org.gravity.ros.analysis.messages.Dataclasses.PublisherInfo;
import org.gravity.ros.analysis.messages.Dataclasses.SubscriberInfo;
import org.gravity.ros.analysis.messages.Dataclasses.TopicInfo;
import org.gravity.ros.analysis.messages.PythonProjectParser;
import org.junit.Test;
import org.python.pydev.parser.jython.ast.FunctionDef;
import org.gravity.ros.analysis.messages.UMLZeichner;

public class TestCallbackClassMultipleAliasing extends TestAbstarctProjectParse {

	
	/**
	 * The logger of this class
	 */
	private static final Logger LOGGER = Logger.getLogger(TestParseMinimalRospy.class);
	private static String projectName = "PyDevCallbackClassMultipleAliasing";
	
	
	public TestCallbackClassMultipleAliasing() throws CoreException {
		super(projectName);
	}

	
	@Test
	public void testParseFunctionShouldReturnTwoAsts(){
		System.out.println("Test project: " + project.getName());
		PythonProjectParser parser = new PythonProjectParser();
		List<AstModuleInfo> parsedList;
		
		try {
			parsedList = parser.parse(project);
			
			assertEquals(2, parsedList.size());
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
			
			//Test for topic "chatter" existance
			assertEquals(1, topicInfo.size());
			assertNotNull(topicInfo.get("chatter"));
			
			//Test for sub existance 
			SubscriberInfo subscriber = ((LinkedList<SubscriberInfo>) topicInfo.get("chatter").subscribers).get(0);
			assertNotNull(subscriber);
			
			//Test for callbackFunction existance
			FunctionInfo callbackFunction = subscriber.callbackFunction;
			assertNotNull(callbackFunction);
			
			//Test for callbackFunction fields
			assertEquals("Subscriber", callbackFunction.folderName);
			assertEquals("callback_alex", callbackFunction.functionName);
			assertEquals("functions.py", callbackFunction.moduleName);
			
		} catch (Exception e) { e.printStackTrace(); }
	}
	
	
//	@Test
//	public void testUML() {
//		PythonProjectParser parser = new PythonProjectParser();
//		List<AstModuleInfo> parsedList;
//		List<FunctionDef> rosAPI;
//		Map<String, TopicInfo> topicInfo;
//		
//		try {
//			parsedList = parser.parse(project);
//			rosAPI = parser.getRosAPI(project);
//			topicInfo = parser.getCalls(parsedList, rosAPI);
//			
//			UMLZeichner.createDiagram(projectName, topicInfo);
//			
//		} catch (Exception e) { e.printStackTrace(); }
//	}
}