// Go to https://www.browxy.com/ paste and run the code : )

package domain;
import java.util.*;

public class HelloWorld {
    
    static double damage = 0.90;
    
    public static void main(String[] args) {
        System.out.println("Hello Max!");
        // reverse order of complexity (have new code above while developing)
        do_primals();
        do_containers();
        do_statements();
        do_loops();
        do_classes();
    }
    
    // Classes
    static void do_classes(){
        System.out.println("########### CLASSES ###########");
        
        // Create user class
        class User {
            int score;
            public boolean hasWon(){
                if ( score >= 100 ){
                    return true;
                } 
                else {
                    return false;
                }
            }
        }
        
        // Init users from user class
        User dau = new User();
        dau.score = 10;
        System.out.println("User has won: " + dau.hasWon());
    }
    
    // Loops
    static void do_loops(){
        System.out.println("########### LOOPS ###########");
        
        // while
        int x = 0;
        while ( x < 3 ) {
            System.out.println("while : " + x);
            x++;
        }
        
        // for
        for (int i = 3; i > 0; i--){
            System.out.println("for: " + i);
        }
        
        // Iterate array of strings
        String[] familyMembers = {"Harry", "Anja", "Konsti", "Max"};
        for (String name : familyMembers){
            System.out.println("Array: " + name);
        }
        
        // Iterate list 
        List<String> familyMembersList = new ArrayList<String>();
        familyMembersList.add("Harry");
        familyMembersList.add("Anja");
        familyMembersList.add("Konsti");
        familyMembersList.add("Max");
        
        for (String name : familyMembersList ){
            System.out.println("List: " + name);
        }
        
    }
    
    // Statements
    static void do_statements(){
        System.out.println("########### STATEMENTS ###########");
        // IF
        int age = 18;
        if ( age == 18 ){
            System.out.println("User is 18.");
        }
        else if ( age < 18 ){
            System.out.println("User is younger than 18.");
        }
        else {
            System.out.println("User is older than 18.");
        }
    }
    
    // Containers
    static void do_containers(){
        System.out.println("########### CONTAINERS ###########");
        // array
        int[] primeNumbers = {2, 3, 5, 7, 11, 13};
        System.out.println(primeNumbers[0]);
        
        // list
        List list = new ArrayList();
        list.add("England");
        list.add("Scotland");
        list.add("Ireland");
        list.add("Wales");
        list.remove(1);
        System.out.println(list);
        
        // hashmap
        Map map = new HashMap();
        map.put("Father", "Rob");
        map.put("Mother", "Kirsten");
        System.out.println("Father is " + map.get("Father"));
    }
    
    // Primals / Data types
    static void do_primals(){
        System.out.println("########### PRIMALS ###########");
        boolean userLikesPizza = true;
        double color = 0.2;
        int age = 10;
        System.out.println(color);
        System.out.println("User likes pizza: " + userLikesPizza);
    }
    
}
